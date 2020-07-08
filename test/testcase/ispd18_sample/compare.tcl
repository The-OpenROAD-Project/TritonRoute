#! /usr/bin/tclsh

set golden_log       [lindex $argv 0]
set test_log         [lindex $argv 1]

set test_wirelength [format "%12d" 0]
set golden_wirelength [format "%12d" 0]
set test_via [format "%12d" 0]
set golden_via [format "%12d" 0]
set test_drc [format "%12d" 0]
set golden_drc [format "%12d" 0]

if {[file exists $golden_log]} {
  catch {set golden_wirelength [format "%12d" [exec grep -e {total wire length = } $golden_log | tail -1 awk {{print $5}}]]}
  catch {set golden_via [format "%12d" [exec grep -e {total number of vias = } $golden_log | tail -1 awk {{print $6}}]]}
  catch {set golden_drc [format "%12d" [exec grep -e {number of violations = } $golden_log | tail -1 awk {{print $5}}]]}
}

if {[file exists $test_log]} {
  catch {set test_wirelength [format "%12d" [exec grep -e {total wire length = } $test_log | tail -1 awk {{print $5}}]]}
  catch {set test_via [format "%12d" [exec grep -e {total number of vias = } $test_log | tail -1 awk {{print $6}}]]}
  catch {set test_drc [format "%12d" [exec grep -e {number of violations = } $test_log | tail -1 awk {{print $5}}]]}
}

if { $golden_wirelength >= $test_wirelength && $golden_via >= $test_via && $golden_drc >= $test_drc } {
  exit 0
} else {
  exit 1
}