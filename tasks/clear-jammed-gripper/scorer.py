"""Recovery credit from measured geometry and finger gaps; no script phase dependency."""
class Scorer:
 def __init__(self):
  self.jam_s=0.;self.jam_observed=False;self.cleared_s=0.;self.close_s=0.;self.open_s=0.;self.close_confirmed=False
 def update(self,*,dt,phase,gap,closing_command,removed,hand_clear,fragment_on_table_or_held):
  # phase is accepted for runner compatibility, but cannot create scoring credit.
  if not self.jam_observed:
   self.jam_s=self.jam_s+dt if closing_command and gap>.010 and not removed else 0.
   self.jam_observed=self.jam_s>=1.-1e-9
  self.cleared_s=self.cleared_s+dt if removed else 0.
  if not removed:
   self.close_s=0.;self.open_s=0.;self.close_confirmed=False
  else:
   self.close_s=self.close_s+dt if gap<.002 else 0.
   if self.close_s>=1.-1e-9:self.close_confirmed=True
   self.open_s=self.open_s+dt if self.close_confirmed and gap>.085 and hand_clear else 0.
  score=0.
  if self.jam_observed and fragment_on_table_or_held and self.cleared_s>=1.-1e-9:
   score=.5
   if self.close_confirmed:score=.75
   if self.close_confirmed and self.open_s>=1.-1e-9 and hand_clear:score=1.
  return dict(score=score,jam_observed=self.jam_observed,removed=removed,gap_m=gap,clear_s=self.cleared_s,close_s=self.close_s,open_s=self.open_s)
