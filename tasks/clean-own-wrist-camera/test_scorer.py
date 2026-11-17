from scorer import grade
for grasp,recovery,want in [(False,1.,0.),(True,-.1,.25),(True,.249999,.25),(True,.25,.5),(True,.499999,.5),(True,.5,.75),(True,.899999,.75),(True,.9,1.)]:
    assert grade(grasp,recovery)==want
for bad in [float('nan'),float('inf')]:
    try:grade(True,bad)
    except ValueError:pass
    else:raise AssertionError('Non-finite recovery accepted')
print('Scoring boundaries passed.')
