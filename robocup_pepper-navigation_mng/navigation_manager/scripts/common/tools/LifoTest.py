#!/usr/bin/python
# -*- coding: utf-8 -*-
 
from Lifo import Lifo


myLifo=Lifo(5)
myLifo.put('A1')
myLifo.put('A2')
myLifo.put('A3')
myLifo.put('A4')
myLifo.put('A5')

print 'Get last insert elt:'+str(myLifo.pop())
print 'Get last insert elt:'+str(myLifo.pop())
print 'Get last insert elt:'+str(myLifo.pop())

print 'Get lifo size:'+str(myLifo.size())

print 'ADD A3,A4,A5,A6,A7'
myLifo.put('A3')
myLifo.put('A4')
myLifo.put('A5')
myLifo.put('A6')
myLifo.put('A7')

print 'Get lifo size:'+str(myLifo.size())

for i in range(myLifo.size()):
    print 'LOOP: Get last elt:'+str(myLifo.pop())

print 'ADD A0,A0,A0,A0,A0'
myLifo.put('A0')
myLifo.put('A0')
myLifo.put('A0')
myLifo.put('A0')
myLifo.put('A0')
print 'ADD B0,B1,B1,B1,B1'
myLifo.put('B0')
myLifo.put('B1')
myLifo.put('B1')
myLifo.put('B1')
myLifo.put('B1')
for i in range(myLifo.size()):
    print 'LOOP: Get last elt:'+str(myLifo.pop())