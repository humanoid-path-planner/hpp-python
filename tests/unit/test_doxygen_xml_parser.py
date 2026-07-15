#!/usr/bin/env python3
#
# Copyright (c) 2026 CNRS
# Author: Paul Sardin
#

import tempfile
import unittest
from pathlib import Path

from doc.doxygen_xml_parser import ClassDoc


DOXYGEN_XML = """\
<doxygen>
  <compounddef>
    <compoundname>hpp::core::Planner</compoundname>
    <briefdescription>
      <para>Class <emphasis>brief</emphasis>.</para>
    </briefdescription>
    <detaileddescription>
      <para>Class <computeroutput>details</computeroutput>.</para>
    </detaileddescription>
    <sectiondef kind="public-func">
      <memberdef kind="function" static="no">
        <name>startSolve</name>
        <briefdescription>
          <para>Start <emphasis>solving</emphasis>.</para>
        </briefdescription>
        <detaileddescription>
          <para>Use the <computeroutput>child planner</computeroutput>.</para>
        </detaileddescription>
      </memberdef>
    </sectiondef>
  </compounddef>
</doxygen>
"""


class TestDoxygenXmlParser(unittest.TestCase):
    def setUp(self):
        self.temporary_directory = tempfile.TemporaryDirectory()
        filename = Path(self.temporary_directory.name) / "planner.xml"
        filename.write_text(DOXYGEN_XML)
        self.class_doc = ClassDoc(filename)

    def tearDown(self):
        self.temporary_directory.cleanup()

    def test_nested_class_description(self):
        self.assertEqual(
            self.class_doc.getClassDoc(), ("Class brief.", "Class details.")
        )

    def test_nested_method_description(self):
        self.assertEqual(
            self.class_doc.getClassMethodDoc("startSolve"),
            ("Start solving.", "Use the child planner.", ["self"]),
        )


if __name__ == "__main__":
    unittest.main()
