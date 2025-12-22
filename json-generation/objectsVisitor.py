# Generated from objects.g4 by ANTLR 4.13.2
from antlr4 import *
if "." in __name__:
    from .objectsParser import objectsParser
else:
    from objectsParser import objectsParser

# This class defines a complete generic visitor for a parse tree produced by objectsParser.

class objectsVisitor(ParseTreeVisitor):

    # Visit a parse tree produced by objectsParser#body.
    def visitBody(self, ctx:objectsParser.BodyContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by objectsParser#objecttype.
    def visitObjecttype(self, ctx:objectsParser.ObjecttypeContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by objectsParser#supertype.
    def visitSupertype(self, ctx:objectsParser.SupertypeContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by objectsParser#attribute.
    def visitAttribute(self, ctx:objectsParser.AttributeContext):
        return self.visitChildren(ctx)


    # Visit a parse tree produced by objectsParser#value.
    def visitValue(self, ctx:objectsParser.ValueContext):
        return self.visitChildren(ctx)



del objectsParser