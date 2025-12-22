# Generated from objects.g4 by ANTLR 4.13.2
from antlr4 import *
if "." in __name__:
    from .objectsParser import objectsParser
else:
    from objectsParser import objectsParser

# This class defines a complete listener for a parse tree produced by objectsParser.
class objectsListener(ParseTreeListener):

    # Enter a parse tree produced by objectsParser#body.
    def enterBody(self, ctx:objectsParser.BodyContext):
        pass

    # Exit a parse tree produced by objectsParser#body.
    def exitBody(self, ctx:objectsParser.BodyContext):
        pass


    # Enter a parse tree produced by objectsParser#objecttype.
    def enterObjecttype(self, ctx:objectsParser.ObjecttypeContext):
        pass

    # Exit a parse tree produced by objectsParser#objecttype.
    def exitObjecttype(self, ctx:objectsParser.ObjecttypeContext):
        pass


    # Enter a parse tree produced by objectsParser#supertype.
    def enterSupertype(self, ctx:objectsParser.SupertypeContext):
        pass

    # Exit a parse tree produced by objectsParser#supertype.
    def exitSupertype(self, ctx:objectsParser.SupertypeContext):
        pass


    # Enter a parse tree produced by objectsParser#attribute.
    def enterAttribute(self, ctx:objectsParser.AttributeContext):
        pass

    # Exit a parse tree produced by objectsParser#attribute.
    def exitAttribute(self, ctx:objectsParser.AttributeContext):
        pass


    # Enter a parse tree produced by objectsParser#value.
    def enterValue(self, ctx:objectsParser.ValueContext):
        pass

    # Exit a parse tree produced by objectsParser#value.
    def exitValue(self, ctx:objectsParser.ValueContext):
        pass



del objectsParser