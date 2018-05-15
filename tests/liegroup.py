from pyhpp.pinocchio import LiegroupSpace, LiegroupElement
import numpy as np

space = LiegroupSpace.R2()
space *= LiegroupSpace.R1()
assert str(space) == "R^2*R^1"

space2 = space * space
space2.mergeVectorSpaces()
assert str(space2) == "R^6"

el1 = LiegroupElement (np.matrix([0.,1.,2.]).T, space)
el2 = LiegroupElement (np.matrix([1.,2.,3.]).T, space)
print "el1\n->",el1
print "el2\n->",el2
v = el2 - el1
print 'v = el2 - el1\n->', v.T

el3 = el1 + v
print "el3 = el1 + v\n->",el3

# __eq__ for LiegroupElement not defined
# print el2 == el3
assert     el2.space() == el3.space()
assert not el2.space() != el3.space()
assert all(el2.v == el3.v)
