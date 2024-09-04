WARNING: sun.reflect.Reflection.getCallerClass is not supported. This will impact performance.

parsing domain file "instance1_domain.pddl" done successfully
parsing problem file "instance1Difficult_problem.pddl" done successfully
# WARNING: Unable to get Instrumentation. Dynamic Attach failed. You may add this JAR as -javaagent manually, or supply -Djdk.attach.allowAttachSelf
# WARNING: Unable to attach Serviceability Agent. You can try again with escalated privileges. Two options: a) use -Djol.tryWithSudo=true to try with sudo; b) echo 0 | sudo tee /proc/sys/kernel/yama/ptrace_scope

problem instantiation done successfully (1312 actions, 221 fluents)

* Starting ENFORCED_HILL_CLIMBING search with FAST_FORWARD heuristic 
* ENFORCED_HILL_CLIMBING search succeeded

found plan as follows:

00: (    fill-box-from-location agent1 box1 valve1 warehouse) [0]
01: (            pick-up-from-location agent1 box1 warehouse) [0]
02: (                             move agent1 warehouse loc3) [0]
03: (                                  move agent1 loc3 loc2) [0]
04: (            deliver-to-workstation agent1 ws2 loc2 box1) [0]
05: (empty-box-workstation agent1 box1 valve1 valve ws2 loc2) [0]
06: (          pick-up-from-workstation agent1 ws2 loc2 box1) [0]
07: (                                  move agent1 loc2 loc3) [0]
08: (                             move agent1 loc3 warehouse) [0]
09: (              deliver-to-location agent1 warehouse box1) [0]
10: (     fill-box-from-location agent1 box1 bolt1 warehouse) [0]
11: (            pick-up-from-location agent1 box1 warehouse) [0]
12: (                             move agent1 warehouse loc1) [0]
13: (            deliver-to-workstation agent1 ws1 loc1 box1) [0]
14: (  empty-box-workstation agent1 box1 bolt1 bolt ws1 loc1) [0]
15: (          pick-up-from-workstation agent1 ws1 loc1 box1) [0]
16: (                             move agent1 loc1 warehouse) [0]
17: (              deliver-to-location agent1 warehouse box1) [0]
18: (    fill-box-from-location agent1 box1 valve2 warehouse) [0]
19: (            pick-up-from-location agent1 box1 warehouse) [0]
20: (                             move agent1 warehouse loc1) [0]
21: (            deliver-to-workstation agent1 ws3 loc1 box1) [0]
22: (empty-box-workstation agent1 box1 valve2 valve ws3 loc1) [0]
23: (                             move agent1 loc1 warehouse) [0]
24: (     fill-box-from-location agent1 box2 tool1 warehouse) [0]
25: (            pick-up-from-location agent1 box2 warehouse) [0]
26: (                             move agent1 warehouse loc1) [0]
27: (            deliver-to-workstation agent1 ws3 loc1 box2) [0]
28: (  empty-box-workstation agent1 box2 tool1 tool ws3 loc1) [0]

time spent:       0.04 seconds parsing 
                  0.32 seconds encoding 
                  2.02 seconds searching
                  2.38 seconds total time

memory used:      4.82 MBytes for problem representation
                  0.00 MBytes for searching
                  4.82 MBytes total


