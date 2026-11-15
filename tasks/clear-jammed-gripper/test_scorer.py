from scorer import Scorer
s=Scorer()
def run(gap,removed,seconds=1.,valid=True,clear=True,closing=True):
 for _ in range(round(seconds/.05)):r=s.update(dt=.05,phase='arbitrary',gap=gap,closing_command=closing,removed=removed,hand_clear=clear,fragment_on_table_or_held=valid)
 return r['score']
assert run(0,True)==0 # cannot earn restoration credit without the initial fault
s=Scorer();assert run(.015,False,closing=False)==0
assert not s.jam_observed
assert run(.015,False)==0
assert s.jam_observed
assert run(.04,True)==.5
assert run(0,True,.6)==.5
assert run(.04,True,.2)==.5
assert run(0,True,.6)==.5 # interrupted intervals cannot be accumulated
assert run(0,True,.5)==.75
assert run(.094,True,clear=False)==.75
assert run(.094,True,seconds=.05,clear=True)==.75
assert run(.094,True,seconds=.95,clear=True)==1.
assert run(.094,True,valid=False)==0
assert run(.04,False)==0
assert run(.094,True)==.5 # renewed clearance requires renewed functional test
print('PASS: partial credit, interrupted holds, fault prerequisite, clearance loss and invalid drop')
