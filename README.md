# RoboLigaFRI
code for autonomous lego robot for uni competition

Ground plan of 1v1 arena
![image](https://github.com/UBevk/RoboLigaFRI/assets/125929632/40e8a8e2-e8c0-4835-aafe-91e30913f7e2)

**Description of a round**

The match is a duel between two robots. Their goal is to collect as many points as possible within a limited time. Points are obtained by collecting unobtainium in their depot, while collecting waste deducts points.

Match duration: up to 3 minutes
Point acquisition:
each piece of ore containing unobtainium in the depot earns one point for the team,
each piece of worthless ore in the depot deducts 2 points,
the robot does not receive information about the quality of the ore piece from the server and must obtain it themselves (using a color sensor, by trial and error),
a piece of ore is considered to be on the depot when the center of the marker is within the depot,
if the tracker fails to recognize the ore marker, the referee decides on scoring. Examples:
the ore piece is overturned and its label is no longer visible,
the ore marker is covered by another object.
Role of charging stations:
at the beginning of the match, the robot has enough fuel for 25 seconds of operation,
the robot refills its fuel at the charging station,
charging, which must last at least 5 seconds, gives it an additional 25 seconds,
the remaining fuel available to the robot is obtained from the server,
if the robot runs out of fuel (time), it must not move until the end of the match,
only one robot can charge at a charging station at a time (first come, first served),
when the robot is at the charging station, the amount of its fuel does not decrease (if it arrives first at the station).
The robot is allowed to move across the entire surface of the arena, including the depots.
Match protocol:
preparation for the match: the competing teams are invited to place their robot in the starting position. The robots must be turned on and connected to the server.
start of the match: the server announces the start of the match with a flag in the match data.
end of the match: the server announces the end of the match with a flag in the match data. Possible ways to end the match:
expiration of time,
both robots run out of fuel,
disqualification of both robots,
at the referee's discretion.
If a robot starts moving before the start of the match, the match is invalidated, and we return to the match preparation. If the robot does this twice within the same match, it is disqualified, and the match is repeated only for the remaining robot.
