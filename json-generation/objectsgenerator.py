import io
import sys
from antlr4 import *
from antlr4.tree.Tree import TerminalNode
from objectsLexer import objectsLexer
from objectsParser import objectsParser
from objectsListener import objectsListener

def main(argv):
    input_stream = FileStream(argv[1])
    lexer = objectsLexer(input_stream)
    stream = CommonTokenStream(lexer)
    parser = objectsParser(stream)
    tree = parser.body()

    if parser.getNumberOfSyntaxErrors() > 0:
        print("syntax errors")
    else:
        print(f"Parse tree: {tree.toStringTree()}")
        print("Walking...")
        ParseTreeWalker().walk(CppListener(), tree)
        ParseTreeWalker().walk(PythonListener(), tree)
    
class CppListener(objectsListener):
    
    def __init__(self):
        super().__init__()
        print(f"Creating C++ listener")
        self._knownns3object = {"JSONMobilityObject"}
        self._filename = {"JSONObject": "json-object", "JSONMobilityObject": "json-mobility-model"}
    
    def enterObjecttype(self, ctx):
        print(f"Entering object type {ctx.name}")
        self._supertypes = []
        self._h_gettersetters = []
        self._h_constants = []
        self._h_members = []
        self._c_constructors = []
        self._c_gettersetters = []
        self._c_constants = []
        self._c_serialize = []
        self._c_deserialize = []

    def exitObjecttype(self, ctx):
        tname = ctx.name.text
        print(f"Writing C++ for {tname}")
        tNAME = tname.upper()

        tfile_name = tname if ctx.FILE_NAME() is None else ctx.FILE_NAME().getText()[1:-1]
        self._filename[tname] = tfile_name

        tsupertypes = [s.name.text for s in ctx.supertype()]
        if len(set(tsupertypes) & self._knownns3object) > 0:
            self._knownns3object.add(tname)

        self._h_constants.append(f"static const std::string {tNAME}_TYPE;")
        self._c_constants.append(f"const std::string {tname}::{tNAME}_TYPE = \"{tname}\";")
        constructor_empty = [f"{supertype}()" for supertype in tsupertypes]
        constructor_json = [f"{supertype}(data)" for supertype in tsupertypes]
        
        # nested here for easy access to object type
        for ctx2 in ctx.attribute():
            aname = ctx2.name.text
            aName = aname[0].upper() + aname[1:]
            aNAME = aname.upper()
            atype = ctx2.type_.text
            avalue = None if ctx2.value() is None or len(ctx2.value()) == 0 else ctx2.value(0).text

            check = None
            cast = None
            if atype == "uint": 
                check = "JSONObject::IsUInt"
            elif atype == "int":
                check = "JSONObject::IsInt"
            elif atype == "float":
                check = "JSONObject::IsNumber" # otherwise, int cannot be cast to float
            elif atype == "bool":
                check = "JSONObject::IsBool"
            elif atype == "string":
                check = "JSONObject::IsString"
            else:
                print(f"Unsupported type {atype}")
                continue

            self._h_members.append(f"{atype} m_{aname};")
            self._h_gettersetters.append(f"{atype} Get{aName}() const;")
            self._c_gettersetters.append(f"{atype} {tname}::Get{aName}() const")
            self._c_gettersetters.append("{")
            self._c_gettersetters.append(f"\treturn m_{aname};")
            self._c_gettersetters.append("}\n")
            self._h_gettersetters.append(f"void Set{aName}({atype} _{aname});")
            self._c_gettersetters.append(f"void {tname}::Set{aName}({atype} _{aname})")
            self._c_gettersetters.append("{")
            self._c_gettersetters.append(f"\tm_{aname} = _{aname};")
            self._c_gettersetters.append("}")

            self._c_deserialize.append( ("else " if len(self._c_deserialize) > 0 else "") + 
                                        f"if (key == {aNAME}_KEY && {check}(key, value))")
            self._c_deserialize.append(f"\tm_{aname} = value.template get<{atype}>();")
            self._c_serialize.append(f"obj.emplace(\"{aname}\", m_{aname});")

            if avalue is not None:
                self._c_constructors.append(f"m_{aname}({avalue})")
            
            self._h_constants.append(f"static const std::string {aNAME}_KEY;")
            self._c_constants.append(f"const std::string {tname}::{aNAME}_KEY = \"{aname}\";")

        with open(tfile_name + ".h", "w") as hfile:
            hfile.write(f"#ifndef {tNAME}_H\n")
            hfile.write(f"#define {tNAME}_H\n\n")

            for supertype in tsupertypes:
              hfile.write(f"#include <ns3/{self._filename[supertype]}.h>\n\n")

            hfile.write("namespace ns3\n{\n")

            hfile.write("// === BEG GENERATED CLASS DECLARATION ===\n")
            extension = (",").join([("public " + s) for s in tsupertypes])
            hfile.write(f"class {tname} : {extension}\n")
            hfile.write("// === END GENERATED CLASS DECLARATION ===\n")
            hfile.write("{\n")

            hfile.write("public:\n\n")

            hfile.write("// === BEG GENERATED CONSTANTS ===\n")
            for _h_constant in self._h_constants:
                hfile.write(f"\t{_h_constant}\n")
            hfile.write("// === END GENERATED CONSTANTS ===\n")

            hfile.write("\n")

            hfile.write("// === BEG GENERATED CONSTRUCTORS ===\n")
            hfile.write(f"\t{tname}();\n")
            hfile.write(f"\t{tname}(const json& data);\n")
            hfile.write("// === END GENERATED CONSTRUCTORS ===\n")

            hfile.write("\n")

            hfile.write("// === BEG GENERATED JSONType ===\n")
            hfile.write("\tvirtual std::string GetJSONType() const;\n")
            hfile.write("// === END GENERATED JSONType ===\n")

            hfile.write("\n")
            
            if tname in self._knownns3object:
              hfile.write("// === BEG GENERATED GetTypeId ===\n")
              hfile.write("\tstatic TypeId GetTypeId();\n")
              hfile.write("// === END GENERATED GetTypeId ===\n")

            hfile.write("\n")

            hfile.write("// === BEG GENERATED GETTERS/SETTERS ===\n")
            for _h_gettersetter in self._h_gettersetters:
                hfile.write(f"\t{_h_gettersetter}\n")
            hfile.write("// === END GENERATED GETTERS/SETTERS ===\n")

            hfile.write("\n")

            hfile.write("protected:\n\n")

            hfile.write("// === BEG GENERATED (DE)SERIALIZATION METHODS ===\n")
            hfile.write("\tvirtual void DoDeserialize(const json& obj);\n")
            hfile.write("\tvirtual void DoSerialize(json& obj) const;\n")
            hfile.write("// === END GENERATED (DE)SERIALIZATION METHODS ===\n")

            hfile.write("\n")

            hfile.write("private:\n\n")

            hfile.write("// === BEG GENERATED MEMBERS ===\n")
            for _h_member in self._h_members:
                hfile.write(f"\t{_h_member}\n")
            hfile.write("// === END GENERATED MEMBERS ===\n")

            hfile.write("\n")

            hfile.write("};\n")
            hfile.write("}\n")
            hfile.write(f"#endif\n")
        
        with open(tfile_name + ".cc", "w") as cfile:
            cfile.write(f"#include \"{tfile_name}.h\"\n")
            
            cfile.write("namespace ns3\n{\n\n")

            cfile.write("// === BEG GENERATED REGISTRATION ===\n")
            cfile.write(f"NS_LOG_COMPONENT_DEFINE(\"{tname}\");\n")
            cfile.write(f"NS_OBJECT_ENSURE_REGISTERED({tname});\n")
            cfile.write("// === END GENERATED REGISTRATION ===\n")

            cfile.write("\n")

            cfile.write("// === BEG GENERATED CONSTANTS ===\n")
            for _c_constant in self._c_constants:
                cfile.write(f"{_c_constant}\n")
            cfile.write("// === END GENERATED CONSTANTS ===\n")

            cfile.write("\n")

            cfile.write("// === BEG GENERATED CONSTRUCTORS ===\n")
            cfile.write(f"{tname}::{tname}()")
            if len(constructor_empty) > 0:
                cfile.write(":"  + ",".join(constructor_empty))
            cfile.write("{}\n\n")
            cfile.write(f"{tname}::{tname}(const json& data)")
            if len(constructor_json) > 0:
                cfile.write(":" + ",".join(constructor_json))
            cfile.write("{}\n\n")
            cfile.write("// === END GENERATED CONSTRUCTORS ===\n")

            cfile.write("\n")

            cfile.write("// === BEG GENERATED GetJSONType ===\n")
            cfile.write(f"std::string {tname}::GetJSONType() const\n") 
            cfile.write("{\n")           
            cfile.write(f"\treturn {tname}::{tNAME}_TYPE;\n")           
            cfile.write("}\n")
            cfile.write("// === END GENERATED GetJSONType ===\n")

            if tname in self._knownns3object:
                cfile.write("// === BEG GENERATED GetTypeId ===\n")
                cfile.write(f"TypeId {tname}::GetTypeId()\n") 
                cfile.write("{\n")           
                parent = tsupertypes[0] if len(tsupertypes)>0 else "JSONMobilityObject"
                cfile.write(f"\tstatic TypeId tid = TypeId(\"ns3::{tname}\").SetParent<{parent}>().SetGroupName(\"V2X\");\n")           
                cfile.write(f"\treturn tid;\n")           
                cfile.write("}\n")
                cfile.write("// === END GENERATED GetTypeId ===\n")

            cfile.write("\n")

            cfile.write("// === BEG GENERATED GETTERS/SETTERS ===\n")
            for _c_gettersetter in self._c_gettersetters:
                cfile.write(f"{_c_gettersetter}\n")
            cfile.write("// === END GENERATED GETTERS/SETTERS ===\n")

            cfile.write("\n")

            cfile.write("// === BEG GENERATED (DE)SERIALIZATION METHODS ===\n")
            cfile.write(f"void {tname}::DoDeserialize(const json& obj)\n")
            cfile.write("{\n")
            for supertype in tsupertypes:
                cfile.write(f"\t{supertype}::DoDeserialize(obj);\n")
            cfile.write("\tfor (json::const_iterator it = obj.begin(); it != obj.end(); ++it)\n")
            cfile.write("\t{\n")
            cfile.write("\t\tstd::string key = it.key();\n")
            cfile.write("\t\tjson value = it.value();\n")
            for s in self._c_deserialize:
                cfile.write(f"\t\t{s}\n")
            cfile.write("\t}\n")
            cfile.write("}\n")   

            cfile.write(f"void {tname}::DoSerialize(json& obj) const\n")
            cfile.write("{\n")
            for supertype in tsupertypes:
                cfile.write(f"\t{supertype}::DoSerialize(obj);\n")
            for s in self._c_serialize:
                cfile.write(f"\t{s}\n")
            cfile.write("}\n")  
            cfile.write("// === END GENERATED (DE)SERIALIZATION METHODS ===\n")

            cfile.write("}\n")
        
class PythonListener(objectsListener):
    
    def __init__(self):
        super().__init__()
        print(f"Creating Python listener")
        self._knownns3object = {"JSONMobilityObject"}
    
    def enterBody(self, ctx: objectsParser.BodyContext):
        self._py_file_fd = open("generated.py", "w")
        self._py_file_fd.write("from .base import *\n\n")

    def exitBody(self, ctx: objectsParser.BodyContext):
        if self._py_file_fd is not None and self._py_file_fd.closed == False:
            self._py_file_fd.close()

    def enterObjecttype(self, ctx):
        print(f"Entering object type {ctx.name}")
        self._supertypes = []
        self._h_gettersetters = []
        self._h_constants = []
        self._h_members = []
        self._c_constructors = []
        self._c_gettersetters = []
        self._c_constants = []
        self._c_serialize = []
        self._c_deserialize = []

    def exitObjecttype(self, ctx):
        tname = ctx.name.text
        print(f"Writing Python for {tname}")
        tNAME = tname.upper()


        tsupertypes = [s.name.text for s in ctx.supertype()]
        if len(set(tsupertypes) & self._knownns3object) > 0:
            self._knownns3object.add(tname)


        constants = []
        constructor_args = []
        constructor_body = ["super().__init__(**kwargs)"]
        serialize_body = []
        deserialize_body = []


        # nested here for easy access to object type
        for ctx2 in ctx.attribute():
            aname = ctx2.name.text
            aName = aname[0].upper() + aname[1:]
            aNAME = aname.upper()
            atype = ctx2.type_.text
            avalue = "None" if ctx2.value() is None or len(ctx2.value()) == 0 else ctx2.value(0).text
            constants.append(f"{aNAME}_KEY = \"{aname}\"")
            constructor_args.append(f"{aname} = {avalue}")
            constructor_body.append(f"self.{aname} = {aname}")
            serialize_body.append(f"obj[{tname}.{aNAME}_KEY] = self.{aname}")
            deserialize_body.append(f"if {tname}.{aNAME}_KEY in obj: self.{aname} = obj[{tname}.{aNAME}_KEY]")
            
        
        extension = (", ").join([s for s in tsupertypes])
        
        self._py_file_fd.write("# === BEG GENERATED CLASS DECLARATION ===\n")
        self._py_file_fd.write(f"class {tname}({extension}):\n")
        self._py_file_fd.write("# === END GENERATED CLASS DECLARATION ===\n")
        
        self._py_file_fd.write("\n")

        self._py_file_fd.write("# === BEG GENERATED CONSTANTS ===\n")
        self._py_file_fd.write(f"\t{tNAME}_TYPE = \"{tname}\"\n")

        for constant in constants:
            self._py_file_fd.write(f"\t{constant}\n")
        self._py_file_fd.write("# === END GENERATED CONSTANTS ===\n")
        
        self._py_file_fd.write("\n")
        
        args = ", ".join(constructor_args)
        if len(constructor_args) > 0:
            args += ", "
        self._py_file_fd.write("# === BEG GENERATED INITIALIZER ===\n")
        self._py_file_fd.write(f"\tdef __init__(self, {args}**kwargs):\n")
        for stmt in constructor_body:
            self._py_file_fd.write(f"\t\t{stmt}\n")
        self._py_file_fd.write("# === END GENERATED INITIALIZER ===\n")
        
        self._py_file_fd.write("\n")

        self._py_file_fd.write("# === BEG GENERATED (DE)SERIALIZATION METHODS ===\n")
        self._py_file_fd.write("\tdef serialize(self, obj):\n")
        self._py_file_fd.write("\t\tsuper().serialize(obj)\n")
        self._py_file_fd.write(f"\t\tobj[JSONObject.JSONOBJECT_TYPE] = {tname}.{tNAME}_TYPE\n")
        if len(serialize_body) > 0:
            self._py_file_fd.write("\t\tif self.alive:\n")
            for stmt in serialize_body:
                self._py_file_fd.write(f"\t\t\t{stmt}\n")
        
        self._py_file_fd.write("\n")

        self._py_file_fd.write("\tdef deserialize(self, obj):\n")
        self._py_file_fd.write("\t\tsuper().deserialize(obj)\n")
        if len(deserialize_body) > 0:
            self._py_file_fd.write("\t\tif obj[JSONObject.JSONOBJECT_ID] == self.id and obj[JSONObject.JSONOBJECT_ALIVE]:\n")
            for stmt in deserialize_body:
                self._py_file_fd.write(f"\t\t\t{stmt}\n")
        
        self._py_file_fd.write("# === END GENERATED (DE)SERIALIZATION METHODS ===\n")

        self._py_file_fd.write("\n")






        

if __name__ == '__main__':
    main(sys.argv)