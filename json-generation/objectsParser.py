# Generated from objects.g4 by ANTLR 4.13.2
# encoding: utf-8
from antlr4 import *
from io import StringIO
import sys
if sys.version_info[1] > 5:
	from typing import TextIO
else:
	from typing.io import TextIO

def serializedATN():
    return [
        4,1,14,59,2,0,7,0,2,1,7,1,2,2,7,2,2,3,7,3,2,4,7,4,1,0,5,0,12,8,0,
        10,0,12,0,15,9,0,1,1,1,1,1,1,1,1,3,1,21,8,1,1,1,1,1,1,1,1,1,5,1,
        27,8,1,10,1,12,1,30,9,1,3,1,32,8,1,1,1,1,1,1,1,1,1,1,1,5,1,39,8,
        1,10,1,12,1,42,9,1,3,1,44,8,1,1,1,1,1,1,2,1,2,1,3,1,3,1,3,1,3,1,
        3,3,3,55,8,3,1,4,1,4,1,4,0,0,5,0,2,4,6,8,0,1,1,0,12,14,60,0,13,1,
        0,0,0,2,16,1,0,0,0,4,47,1,0,0,0,6,49,1,0,0,0,8,56,1,0,0,0,10,12,
        3,2,1,0,11,10,1,0,0,0,12,15,1,0,0,0,13,11,1,0,0,0,13,14,1,0,0,0,
        14,1,1,0,0,0,15,13,1,0,0,0,16,20,5,10,0,0,17,18,5,2,0,0,18,19,5,
        9,0,0,19,21,5,3,0,0,20,17,1,0,0,0,20,21,1,0,0,0,21,22,1,0,0,0,22,
        31,5,4,0,0,23,28,3,4,2,0,24,25,5,6,0,0,25,27,3,4,2,0,26,24,1,0,0,
        0,27,30,1,0,0,0,28,26,1,0,0,0,28,29,1,0,0,0,29,32,1,0,0,0,30,28,
        1,0,0,0,31,23,1,0,0,0,31,32,1,0,0,0,32,33,1,0,0,0,33,34,5,5,0,0,
        34,43,5,4,0,0,35,40,3,6,3,0,36,37,5,6,0,0,37,39,3,6,3,0,38,36,1,
        0,0,0,39,42,1,0,0,0,40,38,1,0,0,0,40,41,1,0,0,0,41,44,1,0,0,0,42,
        40,1,0,0,0,43,35,1,0,0,0,43,44,1,0,0,0,44,45,1,0,0,0,45,46,5,5,0,
        0,46,3,1,0,0,0,47,48,5,10,0,0,48,5,1,0,0,0,49,50,5,10,0,0,50,51,
        5,7,0,0,51,54,5,10,0,0,52,53,5,1,0,0,53,55,3,8,4,0,54,52,1,0,0,0,
        54,55,1,0,0,0,55,7,1,0,0,0,56,57,7,0,0,0,57,9,1,0,0,0,7,13,20,28,
        31,40,43,54
    ]

class objectsParser ( Parser ):

    grammarFileName = "objects.g4"

    atn = ATNDeserializer().deserialize(serializedATN())

    decisionsToDFA = [ DFA(ds, i) for i, ds in enumerate(atn.decisionToState) ]

    sharedContextCache = PredictionContextCache()

    literalNames = [ "<INVALID>", "'='", "'['", "']'", "'('", "')'", "','", 
                     "':'" ]

    symbolicNames = [ "<INVALID>", "<INVALID>", "LBRA", "RBRA", "LPAR", 
                      "RPAR", "COMMA", "COLUMN", "COMMENT", "FILE_NAME", 
                      "ID", "WS", "STRING", "INTEGER", "FLOAT" ]

    RULE_body = 0
    RULE_objecttype = 1
    RULE_supertype = 2
    RULE_attribute = 3
    RULE_value = 4

    ruleNames =  [ "body", "objecttype", "supertype", "attribute", "value" ]

    EOF = Token.EOF
    T__0=1
    LBRA=2
    RBRA=3
    LPAR=4
    RPAR=5
    COMMA=6
    COLUMN=7
    COMMENT=8
    FILE_NAME=9
    ID=10
    WS=11
    STRING=12
    INTEGER=13
    FLOAT=14

    def __init__(self, input:TokenStream, output:TextIO = sys.stdout):
        super().__init__(input, output)
        self.checkVersion("4.13.2")
        self._interp = ParserATNSimulator(self, self.atn, self.decisionsToDFA, self.sharedContextCache)
        self._predicates = None




    class BodyContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def objecttype(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(objectsParser.ObjecttypeContext)
            else:
                return self.getTypedRuleContext(objectsParser.ObjecttypeContext,i)


        def getRuleIndex(self):
            return objectsParser.RULE_body

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterBody" ):
                listener.enterBody(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitBody" ):
                listener.exitBody(self)




    def body(self):

        localctx = objectsParser.BodyContext(self, self._ctx, self.state)
        self.enterRule(localctx, 0, self.RULE_body)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 13
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            while _la==10:
                self.state = 10
                self.objecttype()
                self.state = 15
                self._errHandler.sync(self)
                _la = self._input.LA(1)

        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ObjecttypeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser
            self.name = None # Token

        def LPAR(self, i:int=None):
            if i is None:
                return self.getTokens(objectsParser.LPAR)
            else:
                return self.getToken(objectsParser.LPAR, i)

        def RPAR(self, i:int=None):
            if i is None:
                return self.getTokens(objectsParser.RPAR)
            else:
                return self.getToken(objectsParser.RPAR, i)

        def ID(self):
            return self.getToken(objectsParser.ID, 0)

        def LBRA(self):
            return self.getToken(objectsParser.LBRA, 0)

        def FILE_NAME(self):
            return self.getToken(objectsParser.FILE_NAME, 0)

        def RBRA(self):
            return self.getToken(objectsParser.RBRA, 0)

        def supertype(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(objectsParser.SupertypeContext)
            else:
                return self.getTypedRuleContext(objectsParser.SupertypeContext,i)


        def attribute(self, i:int=None):
            if i is None:
                return self.getTypedRuleContexts(objectsParser.AttributeContext)
            else:
                return self.getTypedRuleContext(objectsParser.AttributeContext,i)


        def COMMA(self, i:int=None):
            if i is None:
                return self.getTokens(objectsParser.COMMA)
            else:
                return self.getToken(objectsParser.COMMA, i)

        def getRuleIndex(self):
            return objectsParser.RULE_objecttype

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterObjecttype" ):
                listener.enterObjecttype(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitObjecttype" ):
                listener.exitObjecttype(self)




    def objecttype(self):

        localctx = objectsParser.ObjecttypeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 2, self.RULE_objecttype)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 16
            localctx.name = self.match(objectsParser.ID)
            self.state = 20
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            if _la==2:
                self.state = 17
                self.match(objectsParser.LBRA)
                self.state = 18
                self.match(objectsParser.FILE_NAME)
                self.state = 19
                self.match(objectsParser.RBRA)


            self.state = 22
            self.match(objectsParser.LPAR)
            self.state = 31
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            if _la==10:
                self.state = 23
                self.supertype()
                self.state = 28
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                while _la==6:
                    self.state = 24
                    self.match(objectsParser.COMMA)
                    self.state = 25
                    self.supertype()
                    self.state = 30
                    self._errHandler.sync(self)
                    _la = self._input.LA(1)



            self.state = 33
            self.match(objectsParser.RPAR)
            self.state = 34
            self.match(objectsParser.LPAR)
            self.state = 43
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            if _la==10:
                self.state = 35
                self.attribute()
                self.state = 40
                self._errHandler.sync(self)
                _la = self._input.LA(1)
                while _la==6:
                    self.state = 36
                    self.match(objectsParser.COMMA)
                    self.state = 37
                    self.attribute()
                    self.state = 42
                    self._errHandler.sync(self)
                    _la = self._input.LA(1)



            self.state = 45
            self.match(objectsParser.RPAR)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class SupertypeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser
            self.name = None # Token

        def ID(self):
            return self.getToken(objectsParser.ID, 0)

        def getRuleIndex(self):
            return objectsParser.RULE_supertype

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterSupertype" ):
                listener.enterSupertype(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitSupertype" ):
                listener.exitSupertype(self)




    def supertype(self):

        localctx = objectsParser.SupertypeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 4, self.RULE_supertype)
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 47
            localctx.name = self.match(objectsParser.ID)
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class AttributeContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser
            self.name = None # Token
            self.type_ = None # Token

        def COLUMN(self):
            return self.getToken(objectsParser.COLUMN, 0)

        def ID(self, i:int=None):
            if i is None:
                return self.getTokens(objectsParser.ID)
            else:
                return self.getToken(objectsParser.ID, i)

        def value(self):
            return self.getTypedRuleContext(objectsParser.ValueContext,0)


        def getRuleIndex(self):
            return objectsParser.RULE_attribute

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterAttribute" ):
                listener.enterAttribute(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitAttribute" ):
                listener.exitAttribute(self)




    def attribute(self):

        localctx = objectsParser.AttributeContext(self, self._ctx, self.state)
        self.enterRule(localctx, 6, self.RULE_attribute)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 49
            localctx.name = self.match(objectsParser.ID)
            self.state = 50
            self.match(objectsParser.COLUMN)
            self.state = 51
            localctx.type_ = self.match(objectsParser.ID)
            self.state = 54
            self._errHandler.sync(self)
            _la = self._input.LA(1)
            if _la==1:
                self.state = 52
                self.match(objectsParser.T__0)
                self.state = 53
                self.value()


        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx


    class ValueContext(ParserRuleContext):
        __slots__ = 'parser'

        def __init__(self, parser, parent:ParserRuleContext=None, invokingState:int=-1):
            super().__init__(parent, invokingState)
            self.parser = parser

        def STRING(self):
            return self.getToken(objectsParser.STRING, 0)

        def INTEGER(self):
            return self.getToken(objectsParser.INTEGER, 0)

        def FLOAT(self):
            return self.getToken(objectsParser.FLOAT, 0)

        def getRuleIndex(self):
            return objectsParser.RULE_value

        def enterRule(self, listener:ParseTreeListener):
            if hasattr( listener, "enterValue" ):
                listener.enterValue(self)

        def exitRule(self, listener:ParseTreeListener):
            if hasattr( listener, "exitValue" ):
                listener.exitValue(self)




    def value(self):

        localctx = objectsParser.ValueContext(self, self._ctx, self.state)
        self.enterRule(localctx, 8, self.RULE_value)
        self._la = 0 # Token type
        try:
            self.enterOuterAlt(localctx, 1)
            self.state = 56
            _la = self._input.LA(1)
            if not((((_la) & ~0x3f) == 0 and ((1 << _la) & 28672) != 0)):
                self._errHandler.recoverInline(self)
            else:
                self._errHandler.reportMatch(self)
                self.consume()
        except RecognitionException as re:
            localctx.exception = re
            self._errHandler.reportError(self, re)
            self._errHandler.recover(self, re)
        finally:
            self.exitRule()
        return localctx





