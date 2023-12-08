#!/usr/bin/env python
#
# Copyright (c) 2018 CNRS
# Authors: Joseph Mirabel
#
#
# This file is part of hpp-python
# hpp-python is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-python is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-python  If not, see
# <http://www.gnu.org/licenses/>.

############# README ###############################
#
# This script takes as arguments an input file and an output file.
#
# 1. Documentation
# ----------------------------------------------------------------
#
# The following commands will be READ (so they must be commented out to avoid
# a compilation error):
#
# - DocNamespace(namespace)
#   :param namespace:
#   Set the name of the current namespace.
#   This namespace is mapped to a package via variable `nsToPackage`.
#
# - DocClass(classname)
#   :param classname:
#   Set the name of the current class being documented.
#
# The following commands will be REPLACED (so they can be put inplace):
#
# - DocClassMethod(methodname,classname)
#   :param methodname:
#   :param classname: optional. If not provided, uses the current class name
#                               defined by DocClass
#   Extract the documentation of this class method from doxygen XML
#   documentation of the package associated with the current namespace.
#
# 2. Example
# ----------------------------------------------------------------
#
#   The following input
#   ```cpp
#   // DocNamespace(hpp::core)
#   // DocClass(Class)
#   .def ("function", Class::function, DocClassMethod(function))
#   ```
#   will be converted into
#   ```cpp
#   // DocNamespace(hpp::core)
#   // DocClass(Class)
#   .def ("function", Class::function, "Documentation of Class::function from install/share/doc/hpp-core/doxygen-xml/...", (arg("self"),arg("input1"),...))
#   ```
#
# 2. Current limitation
# ----------------------------------------------------------------
#
# - It is not possible to disambiguate the prototype in case of overloaded function.
#
# - The detailed description of the function is not well formatted.
#
############# README ###############################

from __future__ import print_function
import re,os,sys
from doxygen_xml_parser import Index

class WrongIndex:
    def __init__ (self, msg):
        self.msg = msg
    def classDoc (self, name):
        return WrongClassDoc (self.msg, name)

class WrongClassDoc:
    def __init__ (self, msg, name):
        self.msg = msg
        self.classname = name

    def _getMsg (self, el):
        msg = self.msg + " " + el + " is not documented"
        print (msg)
        return msg

    def getClassDoc (self):
        return self._getMsg(self.classname), ""

    def getClassMemberDoc (self, member):
        return self._getMsg(self.classname + "::" + member), ""

    def getClassMethodDoc (self, method):
        return self._getMsg(self.classname + "::" + method), "", []

def _xmlDirFromPkgConfig (pkg):
    path = os.popen("pkg-config --variable=docdir " + pkg).read().strip()
    path = os.path.join (path, "doxygen-xml") 
    if not os.path.isdir(path):
      return WrongIndex("Cannot find doxygen-xml for package " + pkg + ".")
    return path

nsToPackage= {
    "hpp::core"       : _xmlDirFromPkgConfig ("hpp-core")                      ,
    "hpp::constraints": _xmlDirFromPkgConfig ("hpp-constraints")
    }

def indexFromNamespace (ns):
  if ns in nsToPackage:
    path = nsToPackage[ns]
    if isinstance(path, str):
        return Index (os.path.join (path, "index.xml"))
    else:
        return path
  raise ValueError ("Unknown namespace " + ns)

def escape (s):
  # return s.replace ('\n', r'\n')
  return s.replace ('\n', r'\n')

def make_doc_string (brief, detailled):
  if len(brief) == 0 or brief.isspace():
    return '"' + detailled + '"'
  return '"' + brief + "\n" + detailled + '"'

def make_args_string (args):
  return '(' + ",".join (['arg("'+a+'")' for a in args]) + ')'

def substitute (istr, ostr):
  nsPattern = re.compile (r"DocNamespace\s*\(\s*(?P<namespace>[\w:]+)\s*\)")
  classPattern = re.compile (r"DocClass\s*\(\s*(?P<class>[\w:]+)\s*\)")
  dcmPattern = re.compile (r"DocClassMethod\s*\(\s*(?P<method>[\w:]+)\s*(,\s*(?P<class>[\w:]+)\s*)?\)")
  classDoc = None
  for line in map (lambda s: s.rstrip(), istr):
    for match in nsPattern.finditer (line):
      currentNamespace = match.group("namespace")
      index = indexFromNamespace (currentNamespace)
    for match in classPattern.finditer (line):
      currentClass = match.group("class")
    for match in dcmPattern.finditer (line):
      if match.group("class") is not None:
        cn = currentNamespace + "::" + match.group("class")
      else:
        cn = currentNamespace + "::" + currentClass
      mn = match.group("method")
      try:
        if classDoc is None or classDoc.classname != cn:
          classDoc = index.classDoc (cn)
        b, d, args = classDoc.getClassMethodDoc (mn)
        line = line.replace (match.group(0),
                escape(make_doc_string(b, d)) + ", "
                + (( make_args_string (args) ) if len(args) > 0 else '""'))
      except IndexError as e:
        print ("Failed to find documentation of method {0}::{1}".format(cn, mn), file=sys.stderr)
        line = line.replace (match.group(0), '"' + str(e) + '", ""')
    print(line, file=ostr)

if __name__ == '__main__':
  srcfile = sys.argv[1]
  with open(srcfile) as istr:
    if len(sys.argv) > 2:
      dstfile = sys.argv[2]
      with open(dstfile, "w") as ostr:
        substitute(istr, ostr)
    else:
      substitute(istr, sys.stdout)
