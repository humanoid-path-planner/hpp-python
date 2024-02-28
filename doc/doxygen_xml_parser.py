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
        return ClassDoc(path.join(self.directory, el.get("refid") + ".xml"))


class ClassDoc:
    def __init__(self, filename):
        self.tree = etree.parse(filename)
        self.compound = self.tree.find("/compounddef")
        self.classname = self.compound.find("compoundname").text.strip()

    @staticmethod
    def _getDoc(el):
        b = el.find("briefdescription")
        d = el.find("detaileddescription")
        return etree.tostring(b, method="text").strip(), d.text.strip()

    def _getMember(self, sectionKind, memberDefKind, name):
        # return self.compound.xpath ("sectiondef[@kind='" + sectionKind
        try:
            return self.compound.xpath(
                "sectiondef[re:test(@kind,'"
                + sectionKind
                + "')]/memberdef[@kind='"
                + memberDefKind
                + "' and name='"
                + name
                + "']",
                namespaces={"re": "http://exslt.org/regular-expressions"},
            )[0]
        except IndexError as e:
            msg = (
                "Error: Could not find member ("
                + sectionKind
                + ") "
                + name
                + " of class "
                + self.classname
            )
            print(msg + "\n" + str(e), file=sys.stderr)
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
        args += [el.text.strip() for el in member.xpath("param/declname")]
        for parameters in dd.xpath("para/parameterlist/parameteritem"):
            pargs = [
                el.text.strip()
                for el in parameters.find("parameternamelist").findall("parametername")
            ]
            pns = " ".join(pargs)
            pd = parameters.find("parameterdescription").find("para").text.strip()
            d += "\n:param " + pns + ":" + pd
        return b, d, args


# index = Index (sys.argv[1])
# classDoc = index.classDoc ("hpp::core::BiRRTPlanner")
# print classDoc.getClassDoc()
# print classDoc.getClassMethodDoc("oneStep")
