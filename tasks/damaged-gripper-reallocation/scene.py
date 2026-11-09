"""Put the apple in the red bowl and the orange in the blue bowl."""
import numpy as np
from yam_arena.task_file import Asset,TABLE_TOP
from yam_arena import station
from yam_arena.demo import pick_place_waypoints,Waypoint
from yam_arena.constants import FINGER_OPEN
SCENE='station';TOP=TABLE_TOP[SCENE]
PICKS={'orange':(.25,-.08),'apple':(.25,.08)}
GOALS={'orange':(.12,-.12),'apple':(.12,.12)}
BOWLS={'orange':'blue_bowl','apple':'red_bowl'}
OBJECTS=[Asset('apple','apple_01_objaverse_robolab',PICKS['apple'],.066,mass=.12,color=station.APPLE_RED,convex=True),Asset('orange','orange_02_fruits_veggies_robolab',PICKS['orange'],.064,mass=.11,color=station.ORANGE),Asset('red_bowl','bowl_ycb_robolab',GOALS['apple'],.166,color=(.75,.12,.12)),Asset('blue_bowl','bowl_ycb_robolab',GOALS['orange'],.166,color=(.12,.25,.75))]
def success(objects):
 from scorer import contained_fraction
 return all(contained_fraction(objects[c],objects[BOWLS[c]])>=.5 and objects[c].resting() for c in GOALS)
def score(objects):return float(success(objects))
def demo():
 path=[]
 for c in ['orange','apple']:
  grip=station.GRIP_HEIGHT[c]
  pts=pick_place_waypoints((*PICKS[c],TOP+grip),(*GOALS[c],TOP+station.TARGET_TOP['bowl']+grip+.015),travel_z=TOP+.12,speed=.7)
  pts[-1].duration=1.5
  pts.append(Waypoint(2.5,(.25,-.15,TOP+.15),FINGER_OPEN,'withdraw after vertical clearance'))
  for w in pts:w.label=c+': '+w.label
  path+=pts+[Waypoint(2.5,(.25,-.15,TOP+.15),FINGER_OPEN,c+': release check')]
 return {'right':path}
