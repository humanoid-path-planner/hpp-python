from pyhpp.pinocchio import LiegroupSpace, LiegroupElement
import numpy as np

space = LiegroupSpace.R2()
space *= LiegroupSpace.R1(False)
assert str(space) == "R^3"

space2 = space * space
space2.mergeVectorSpaces()
assert str(space2) == "R^6"

el1 = LiegroupElement (np.array([0.,1.,2.]), space)
el2 = LiegroupElement (np.array([1.,2.,3.]), space)
print(f"el1\n->{el1}")
print(f"el2\n->{el2}")
v = el2 - el1
print(f"v = el2 - el1\n->{v.T}")

el3 = el1 + v
print(f"el3 = el1 + v\n->{el3}")

# __eq__ for LiegroupElement not defined
assert     el2.space() == el3.space()
assert not el2.space() != el3.space()
assert all(el2.v == el3.v)
