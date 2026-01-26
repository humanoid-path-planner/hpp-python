from __future__ import print_function
from lxml import etree
from os import path
import sys


class Index:
    def __init__(self, filename):
        self.tree = etree.parse(filename)
        self.directory = path.dirname(filename)

    def _getCompound(self, kind, name):
        return self.tree.xpath(
            "/doxygenindex/compound[@kind='" + kind + "' and name='" + name + "']"
        )[0]

    def classDoc(self, name):
        el = self._getCompound("class", name)
        return ClassDoc(
            path.join(self.directory, el.get("refid") + ".xml"), self.directory
        )


class ClassDoc:
    def __init__(self, filename, directory=None):
        self.tree = etree.parse(filename)
        self.compound = self.tree.find("./compounddef")
        self.classname = self.compound.find("compoundname").text.strip()
        self.directory = directory if directory else path.dirname(filename)
        self._group_cache = {}

    @staticmethod
    def _getDoc(el):
        b = el.find("briefdescription")
        d = el.find("detaileddescription")
        brief = etree.tostring(b, method="text", encoding="unicode").strip()
        detailed = d.text.strip() if d.text else ""
        return brief, detailed

    def _getMemberFromGroup(self, memberDefKind, name):
        """Look for memberdef in a group file via member refid."""
        members = self.compound.xpath(
            "sectiondef/member[@kind='" + memberDefKind + "' and name='" + name + "']"
        )
        if not members:
            return None
        member = members[0]
        refid = member.get("refid")
        if not refid:
            return None
        group_id = refid.rsplit("_1", 1)[0]
        if group_id not in self._group_cache:
            group_file = path.join(self.directory, group_id + ".xml")
            if not path.isfile(group_file):
                return None
            self._group_cache[group_id] = etree.parse(group_file)
        group_tree = self._group_cache[group_id]
        memberdefs = group_tree.xpath(
            "//memberdef[@id='" + refid + "' and @kind='" + memberDefKind + "']"
        )
        if memberdefs:
            return memberdefs[0]
        return None

    def _getMemberFromInherited(self, memberDefKind, name):
        """Look for memberdef in base class XML via listofallmembers member refid."""
        members = self.compound.xpath("listofallmembers/member[name='" + name + "']")
        if not members:
            return None
        member = members[0]
        refid = member.get("refid")
        if not refid:
            return None
        # Extract base class file from refid
        # e.g., "classhpp_1_1pinocchio_1_1AbstractDevice_1a312..." -> "classhpp_1_1pinocchio_1_1AbstractDevice"
        parts = refid.rsplit("_1", 1)
        if len(parts) < 2:
            return None
        base_class_id = parts[0]
        base_class_file = path.join(self.directory, base_class_id + ".xml")
        if not path.isfile(base_class_file):
            return None
        if base_class_id not in self._group_cache:
            self._group_cache[base_class_id] = etree.parse(base_class_file)
        base_tree = self._group_cache[base_class_id]
        memberdefs = base_tree.xpath(
            "//memberdef[@id='" + refid + "' and @kind='" + memberDefKind + "']"
        )
        if memberdefs:
            return memberdefs[0]
        return None

    def _getMember(self, sectionKind, memberDefKind, name):
        # First try to find memberdef directly in class XML
        results = self.compound.xpath(
            "sectiondef[re:test(@kind,'"
            + sectionKind
            + "')]/memberdef[@kind='"
            + memberDefKind
            + "' and name='"
            + name
            + "']",
            namespaces={"re": "http://exslt.org/regular-expressions"},
        )
        if results:
            return results[0]
        # Fall back to looking in group files (for @ingroup documented classes)
        memberdef = self._getMemberFromGroup(memberDefKind, name)
        if memberdef is not None:
            return memberdef
        # Fall back to looking in base class files (for inherited methods)
        memberdef = self._getMemberFromInherited(memberDefKind, name)
        if memberdef is not None:
            return memberdef
        msg = (
            "Error: Could not find member ("
            + sectionKind
            + ") "
            + name
            + " of class "
            + self.classname
        )
        print(msg, file=sys.stderr)
        raise IndexError(msg)

    def getClassDoc(self):
        return self._getDoc(self.compound)

    def getClassMemberDoc(self, membername):
        # member = self.compound.find ("sectiondef[@kind='public-attrib']/memberdef[@kind='variable' and name='" + methodname + "']")
        member = self._getMember("public-attrib", "variable", membername)
        return self._getDoc(member)

    def getClassMethodDoc(self, methodname):
        # member = self.compound.xpath ("sectiondef[@kind='public-func']/memberdef[@kind='function' and name='" + methodname + "']")[0]
        # member = self._getMember ('.*-func', 'function', methodname)
        member = self._getMember(".*", "function", methodname)
        b, d = self._getDoc(member)
        dd = member.find("detaileddescription")
        if member.get("static") == "no":
            args = [
                "self",
            ]
        else:
            args = []
        args += [el.text.strip() for el in member.xpath("param/declname") if el.text]
        for parameters in dd.xpath("para/parameterlist/parameteritem"):
            pargs = [
                el.text.strip()
                for el in parameters.find("parameternamelist").findall("parametername")
                if el.text
            ]
            pns = " ".join(pargs)
            para_el = parameters.find("parameterdescription").find("para")
            pd = para_el.text.strip() if para_el is not None and para_el.text else ""
            d += "\n:param " + pns + ":" + pd
        return b, d, args


# index = Index (sys.argv[1])
# classDoc = index.classDoc ("hpp::core::BiRRTPlanner")
# print classDoc.getClassDoc()
# print classDoc.getClassMethodDoc("oneStep")
