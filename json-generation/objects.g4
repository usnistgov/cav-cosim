grammar objects;

LBRA: '[';
RBRA: ']';
LPAR: '(';
RPAR: ')';
COMMA: ',';
COLUMN: ':';

COMMENT : '//' ~[\r\n]* -> skip;

FILE_NAME: '\'' [-a-zA-Z0-9_.]*[-a-zA-Z0-9_] '\''; 

ID: [a-zA-Z_][a-zA-Z0-9_]*;

WS: [ \r\n\t]+ -> skip;

STRING: '"' ( ([\\] .) | ~[\\\r\n\f"])* '"';

INTEGER: [+-]? INT_FRAG;

FLOAT: [+-]? EXPONENT_FRAG|DECIMAL_FRAG;

fragment EXPONENT_FRAG: (INT_FRAG|DECIMAL_FRAG) [eE] [+-]? [0-9]+;

fragment DECIMAL_FRAG: INT_FRAG? '.' [0-9]*;
fragment INT_FRAG: [0]+ | [1-9][0-9]*;


body: objecttype*;

objecttype: 
  name=ID
  (LBRA FILE_NAME RBRA)?
  LPAR (supertype (COMMA supertype)*)? RPAR
  LPAR (attribute (COMMA attribute)*)? RPAR;

supertype: name=ID;

attribute: name=ID COLUMN type=ID ('=' value)?;

value : STRING|INTEGER|FLOAT;
