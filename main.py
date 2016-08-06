#!/usr/bin/python3
# -*- encoding: utf-8 -*-
from functools import cmp_to_key
import copy
from enum import Enum
import fractions
import math
from decimal import *
getcontext().prec = 100

class Clockwise(Enum):
  ccw = 1
  clockwise = -1
  cab = 2
  abc = -2
  otherwise = 0

class INOUT(Enum):
  IN = 1
  OUT = -1
  ON = 0

class Point:
  def frac2decimal(s):
    x = s.find("/")
    if x == -1:
      return Decimal(s)
    else:
      a = Decimal(s[:x] + ".0")
      b = Decimal(s[x + 1:]+ ".0")
      return a / b

  def __init__(self, x, y):
    if isinstance(x, str) and isinstance(y, str):
      self.x = Point.frac2decimal(x)
      self.y = Point.frac2decimal(y)
    elif isinstance(x, Decimal) and isinstance(y, Decimal):
      self.x = x
      self.y = y
    elif (isinstance(x, float) and isinstance(y, float)) or (isinstance(x, int) and isinstance(y, int)):
      self.x = Decimal(x)
      self.y = Decimal(y)
    else:
      assert False

  def __repr__(self):
    return "({}, {})".format(self.x, self.y)

  def to_s(self):
    return "{},{}".format(self.x, self.y)

  def __eq__(self, other):
    return (self.x == other.x) and (self.y == other.y)

  def __add__(self, other):
    return Point(self.x + other.x, self.y + other.y)

  def __sub__(self, other):
    return Point(self.x -  other.x, self.y - other.y)

  def __mul__(self, other):
    if isinstance(other, Point):
      return (self.x * other.x) + (self.y * other.y)
    elif isinstance(other, Decimal):
      return Point(self.x * other, self.y * other)
    else:
      print(self, other)
      assert False

  def __truediv__(self, other):
    if isinstance(other, Decimal):
      return Point(self.x /other, self.y / other)
    elif isinstance(other, float):
      other0 = Decimal(other)
      return Point(self.x /other0, self.y / other0)
    else:
      print(self, other)
      assert False

  def norm(self):
    return (self.x ** 2 + self.y ** 2).sqrt()

  def regular(self):
    return self / (self.x ** 2 + self.y ** 2).sqrt()

  def cross(p1, p2):
    return (p1.x * p2.y) - (p1.y * p2.x)

  def dot(p1, p2):
    return (p1.x * p2.x) + (p1.y * p2.y)

  def ccw(p1, p2, p3):
    a = p1
    b = p2 - p1
    c = p3 - p1
    if Point.cross(b, c) > 0:
      return Clockwise.ccw
    if Point.cross(b, c) < 0:
      return Clockwise.clockwise
    if Point.dot(b, c) < 0:
      return Clockwise.cab
    if b.norm() < c.norm():
      return Clockwise.abc
    return Clockwise.otherwise

class Line:
  def __init__(self, p1, p2):
    self.p1 = p1
    self.p2 = p2

  def __repr__(self):
    return "({}, {})".format(self.p1, self.p2)

  def ccw(self, p):
    return Point.ccw(self.p1, self.p2, p)

  def projection(self, p):
    a = p - self.p1
    b = self.p2 - self.p1
    t = (Point.dot(a, b) / b.norm()) / b.norm()
    return self.p1 + b * t

  def lin_sym(self, p):
    d = self.projection(p) - p
    return p + d * Decimal(2)

class Paper:
  def __init__(self, vs):# TODO: vsが反時計回りであること
    self.vertex = Paper.vertex_sort(vs)
    self.skeltons = []
    self.in_vertex = []

  def __repr__(self):
    return "({}, {})".format(self.vertex, self.skeltons)

  def contains(self, p):
    in_flg = False
    for i in range(len(self.vertex)):
      a = self.vertex[i] - p
      b = self.vertex[(i + 1) % len(self.vertex)]
      if a.x > b.x:
        t = b
        b = a
        a = b
      if a.x <= 0 and 0 < b.x:
        if Point.cross(a, b) < 0:
          in_flg = not in_flg
      if Point.cross(a, b) == 0 and Point.dot(a, b) <= 0:
        return INOUT.ON
    if in_flg:
      return INOUT.IN
    else:
      return INOUT.OUT

  def vertex_sort(vs):
    center = Point(0, 0)
    for v in vs:
      center = center + v
    ini = Point(0, 0)
    center = center / (len(vs) + 0.0)
    base = (ini - center).regular()
    baseline = Line(center, ini)

    def angle(p1):
      if p1 == ini:
        return 0
      ccw = baseline.ccw(p1)
      cos = Point.dot(base, (p1 - center).regular())
      theta = math.acos(cos)
      if ccw == Clockwise.clockwise:
        theta = 2 * math.acos(-1) - theta
      return theta
    #print([(v, angle(v), baseline.ccw(v)) for v in vs])
    srted = sorted(vs, key=angle)
    ret = []
    for v in srted:
      if not (v in ret):
        ret.append(v)
    return ret

  def fold(self, line, v):
    center = Point("1/2", "1/2")
    d = line.projection(v) - v
    v_ccw = line.ccw(v)
    assert(v_ccw == Clockwise.ccw or v_ccw == Clockwise.clockwise)
    new_vs = []
    new_in_vs = []
    for v in self.vertex:
      if line.ccw(v) != v_ccw:
        new_vs.append(v)
      else:
        n_v = line.lin_sym(v)
        if not self.contains(n_v) != INOUT.IN:# TODO: 既存の頂点とONの時に注意
          new_vs.append(n_v)
        else:
          pass # TODO 内部の点 new_in_vs.append(n_v)
    new_vs.append(line.p1)
    new_vs.append(line.p2)
    self.vertex = Paper.vertex_sort(new_vs)
    self.skeltons.append(line)

class Origami:
  def __init__(self, init=None):
    if init == None:
      self.sv = [Point(0, 0), Point(1, 0),
                 Point(1, 1), Point(0, 1)]
      self.dv = [Point(0, 0), Point(1, 0),
                 Point(1, 1), Point(0, 1)]
      self.fs = [[0, 1, 2, 3]]
    else:
      assert False
    assert (len(self.sv) == len(self.dv))

  def to_s(self):
    def ln(s):
      return str(s) + "\n"
    out = ""
    out += ln(len(self.sv))
    for v in self.sv:
      out += ln(v.to_s())
    out += ln(len(self.fs))
    for f in self.fs:
      s = str(len(f))
      for i in f:
        s += " " + str(i)
      out += ln(s)
    for v in self.dv:
      out += ln(v.to_s())
    return out

  def __repr__(self):
    return "(src={}, dst={}, facets={})".format(self.sv, self.dv, self.fs)

def main():
  origami = Origami()
  print(origami.to_s())

if __name__ == '__main__':
  main()
