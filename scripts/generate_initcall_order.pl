#!/usr/bin/env perl
# SPDX-License-Identifier: GPL-2.0
#
# Generates a linker script that specifies the correct initcall order.
#
# Copyright (C) 2019 Google LLC

use strict;
use warnings;
use IO::Handle;

my $nm = $ENV{'LLVM_NM'} || "llvm-nm";
my $ar = $ENV{'AR'}	 || "llvm-ar";
my $objtree = $ENV{'objtree'} || ".";

## list of all object files to process, in link order
my @objects;
## currently active child processes
my $jobs = {};		# child process pid -> file handle
## results from child processes
my $results = {};	# object index -> { level, function }
my $early_results = {};
## reads _NPROCESSORS_ONLN to determine the number of processes to start
sub get_online_processors {
	open(my $fh, "getconf _NPROCESSORS_ONLN 2>/dev/null |")
		or die "$0: failed to execute getconf: $!";
	my $procs = <$fh>;
	close($fh);

	if (!($procs =~ /^\d+$/)) {
		return 1;
	}

	return int($procs);
}

## finds initcalls defined in an object file, parses level and function name,
## and prints it out to the parent process
sub find_initcalls {
	my ($object) = @_;

	die "$0: object file $object doesn't exist?" if (! -f $object);

	open(my $fh, "\"$nm\" --just-symbol-name --defined-only \"$object\" 2>/dev/null |")
		or die "$0: failed to execute \"$nm\": $!";

	my $initcalls = {};

	while (<$fh>) {
		chomp;

		my ($counter, $line, $symbol) = $_ =~ /^__initcall_(\d+)_(\d+)_(.*)$/;

		if (!defined($counter) || !defined($line) || !defined($symbol)) {
			next;
		}

		my ($function, $level) = $symbol =~
			/^(.*)((early|rootfs|con|security|[0-9])s?)$/;

		die "$0: duplicate initcall counter value in object $object: $_"
			if exists($initcalls->{$counter});

		$initcalls->{$counter} = {
			'level'    => $level,
			'line'     => $line,
			'function' => $function
		};
	}

	close($fh);

	# sort initcalls in each object file numerically by the counter value
	# to ensure they are in the order they were defined
	foreach my $counter (sort { $a <=> $b } keys(%{$initcalls})) {
		print $initcalls->{$counter}->{"level"} . " " .
		      $counter . " " .
		      $initcalls->{$counter}->{"line"} . " " .
		      $initcalls->{$counter}->{"function"} . "\n";
	}
}

## finds early_initcalls defined in an object file, parses level and function name,
## and prints it out to the parent process
sub find_early_initcalls {
	my ($object) = @_;

	die "$0: object file $object doesn't exist?" if (! -f $object);

	open(my $fh, "\"$nm\" --just-symbol-name --defined-only \"$object\" 2>/dev/null |")
		or die "$0: failed to execute \"$nm\": $!";

	my $early_initcalls = {};

	while (<$fh>) {
		chomp;

		my ($early_subsys, $early_counter, $early_line, $early_symbol) = $_ =~ /^__early(\d+)_initcall_(\d+)_(\d+)_(.*)$/;

		if (!defined($early_subsys) || !defined($early_counter) ||
			!defined($early_line) || !defined($early_symbol)) {
			next;
		}

		my ($early_function, $early_level) = $early_symbol =~
			/^(.*)(\d+)$/;

		die "$0: duplicate initcall counter value in object $object: $_"
			if exists($early_initcalls->{$early_counter});

		$early_initcalls->{$early_counter} = {
			'subsys'   => $early_subsys,
			'level'    => $early_level,
			'line'     => $early_line,
			'function' => $early_function
		};
	}

	close($fh);

	# sort initcalls in each object file numerically by the counter value
	# to ensure they are in the order they were defined
	foreach my $early_counter (sort { $a <=> $b } keys(%{$early_initcalls})) {
		print $early_initcalls->{$early_counter}->{"subsys"} . "-" .
		      $early_initcalls->{$early_counter}->{"level"} . "-" .
		      $early_counter . "-" .
		      $early_initcalls->{$early_counter}->{"line"} . "-" .
		      $early_initcalls->{$early_counter}->{"function"} . "\n";
	}
}

## waits for any child process to complete, reads the results, and adds them to
## the $results array for later processing
sub wait_for_results {
	my $pid = wait();
	if ($pid > 0) {
		my $fh = $jobs->{$pid};

		# the child process prints out results in the following format:
		#  line 1:    <object file index>
		#  initcalls line:
		#  line 2..n: <level> <counter> <line> <function>
		#  early-initcall line:
		#  line 2..n: <subsys>-<level>-<counter>-<line>-<function>

		my $index = <$fh>;
		chomp($index);

		if (!($index =~ /^\d+$/)) {
			die "$0: child $pid returned an invalid index: $index";
		}
		$index = int($index);

		while (<$fh>) {
			chomp;
			my ($early_subsys, $early_level, $early_counter,
				$early_line, $early_function) = $_ =~
				/^(\d+)\-(\d+)\-(\d+)\-(\d+)\-(.*)$/;

			my ($level, $counter, $line, $function) = $_ =~
				/^([^\ ]+)\ (\d+)\ (\d+)\ (.*)$/;

			if ((!defined($level) ||
				!defined($counter) ||
				!defined($line) ||
				!defined($function)) &&
				(!defined($early_subsys) ||
				 !defined($early_level) ||
				 !defined($early_counter) ||
				 !defined($early_line) ||
				 !defined($early_function))){
				die "$0: child $pid returned invalid data";
			}

			if(defined($level)) {
				if (!exists($results->{$index})) {
					$results->{$index} = [];
				}

				push (@{$results->{$index}}, {
					'level'    => $level,
					'counter'  => $counter,
					'line'     => $line,
					'function' => $function
				});
			} elsif(defined($early_level)) {
				if (!exists($early_results->{$index})) {
					$early_results->{$index} = [];
				}

				push (@{$early_results->{$index}}, {
					'subsys'   => $early_subsys,
					'level'    => $early_level,
					'counter'  => $early_counter,
					'line'     => $early_line,
					'function' => $early_function
				});
			}
		}

		close($fh);
		delete($jobs->{$pid});
	}
}

## launches child processes to find initcalls from the object files, waits for
## each process to complete and collects the results
sub process_objects {
	my $index = 0;	# link order index of the object file
	my $njobs = get_online_processors();

	while (scalar(@objects) > 0) {
		my $object = shift(@objects);

		# fork a child process and read it's stdout
		my $pid = open(my $fh, '-|');

		if (!defined($pid)) {
			die "$0: failed to fork: $!";
		} elsif ($pid) {
			# save the child process pid and the file handle
			$jobs->{$pid} = $fh;
		} else {
			STDOUT->autoflush(1);
			print "$index\n";
			find_initcalls("$objtree/$object");
			find_early_initcalls("$objtree/$object");
			exit;
		}

		$index++;

		# if we reached the maximum number of processes, wait for one
		# to complete before launching new ones
		if (scalar(keys(%{$jobs})) >= $njobs && scalar(@objects) > 0) {
			wait_for_results();
		}
	}

	# wait for the remaining children to complete
	while (scalar(keys(%{$jobs})) > 0) {
		wait_for_results();
	}
}

## gets a list of actual object files from thin archives, and adds them to
## @objects in link order
sub find_objects {
	while (my $file = shift(@ARGV)) {
		my $pid = open (my $fh, "\"$ar\" t \"$file\" 2>/dev/null |")
			or die "$0: failed to execute $ar: $!";

		my @output;

		while (<$fh>) {
			chomp;
			push(@output, $_);
		}

		close($fh);

		# if $ar failed, assume we have an object file
		if ($? != 0) {
			push(@objects, $file);
			next;
		}

		# if $ar succeeded, read the list of object files
		foreach (@output) {
			push(@objects, $_);
		}
	}
}

## START
find_objects();
process_objects();

## process results and add them to $sections in the correct order
my $sections = {};
my %early_sections;

foreach my $index (sort { $a <=> $b } keys(%{$results})) {
	foreach my $result (@{$results->{$index}}) {
		my $level = $result->{'level'};

		if (!exists($sections->{$level})) {
			$sections->{$level} = [];
		}

		my $fsname = $result->{'counter'} . '_' .
			     $result->{'line'}    . '_' .
			     $result->{'function'};

		push(@{$sections->{$level}}, $fsname);
	}
}

if (!keys(%{$sections})) {
	exit(0); # no initcalls...?
}

foreach my $index (sort { $a <=> $b } keys(%{$early_results})) {
	foreach my $early_result (@{$early_results->{$index}}) {
		my $subsys = $early_result->{'subsys'};
		my $level = $early_result->{'level'};

		if(!exists($early_sections{$subsys}{$level})) {
			$early_sections{$subsys}{$level} = [];
		}

		my $fsname = $early_result->{'counter'} . '_' .
			     $early_result->{'line'}    . '_' .
			     $early_result->{'function'};

		push(@{$early_sections{$subsys}{$level}}, $fsname);
	}
}


## print out a linker script that defines the order of initcalls for each
## level
print "SECTIONS {\n";

foreach my $level (sort(keys(%{$sections}))) {
	my $section;

	if ($level eq 'con') {
		$section = '.con_initcall.init';
	} elsif ($level eq 'security') {
		$section = '.security_initcall.init';
	} else {
		$section = ".initcall${level}.init";
	}

	print "\t${section} : {\n";

	foreach my $fsname (@{$sections->{$level}}) {
		print "\t\t*(${section}..${fsname}) ;\n"
	}

	print "\t}\n";
}

foreach my $subsys (sort(keys(%early_sections))) {
	foreach my $level (sort(keys(%{$early_sections{$subsys}}))) {
		my $section = ".early${subsys}.initcall${level}.init";

		print "\t${section} : {\n";

		foreach my $fsname (@{${early_sections}{$subsys}{$level}}) {
			print "\t\t*(${section}..${fsname}) ;\n"
		}
		print "\t}\n";
	}

}

print "}\n";
