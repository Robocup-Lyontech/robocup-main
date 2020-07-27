#!/usr/bin/python
# -*- coding: utf-8 -*-
 
class Lifo(object):
    """Lifo: last in first out, if Lifo is full elemnt at the end of the lifo is removed"""
    _lifo=[]
    _maxSize=5
 
    def __init__(self,maxSize=None):
        self._lifo=[]
        self._maxSize = maxSize

    def put(self,obj):
        if(len(self._lifo)<self._maxSize):
            self._lifo.append(obj)
        else:
            tmpLifo=[]
            for i in range(len(self._lifo)):
                if(i==0):
                    #do nothing elt is removed
                    pass
                else:
                    tmpLifo.append(self._lifo[i])
            #add new elt
            tmpLifo.append(obj)
            self._lifo=tmpLifo

    def pop(self):
        if len(self._lifo)==0:
            raise ValueError ("_lifo is empty")
        return self._lifo.pop()
 
    def size(self):
        return len(self._lifo)
    
    def isEmpty(self):
        if len(self._lifo)==0:
            return True
        else:
            return False
    def removeAll(self):
        self._lifo=[]

    def get(self,index):
        if index <len(self._lifo) and index >= 0:
            return self._lifo[index]
        else:
            raise ValueError ("index out of range :"+str(index))
