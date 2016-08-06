#!/usr/bin/python3
# -*- encoding: utf-8 -*-
from functools import cmp_to_key
import copy
from enum import IntEnum, Enum
import fractions
import math
from fractions import Fraction

class Clockwise(IntEnum):
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
  def __init__(self, x, y):
    if isinstance(x, str) and isinstance(y, str):
      self.x = Fraction(x)
      self.y = Fraction(y)
    elif isinstance(x, Fraction) and isinstance(y, Fraction):
      self.x = x
      self.y = y
    elif (isinstance(x, float) and isinstance(y, float)) or (isinstance(x, int) and isinstance(y, int)):
      self.x = Fraction(x)
      self.y = Fraction(y)
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
    elif isinstance(other, Fraction):
      return Point(self.x * other, self.y * other)
    else:
      print(self, other)
      assert False

  def __truediv__(self, other):
    if isinstance(other, Fraction):
      return Point(self.x /other, self.y / other)
    elif isinstance(other, float):
      other0 = Fraction(other)
      return Point(self.x /other0, self.y / other0)
    else:
      print(self, other)
      assert False

  def norm2(self):
    return (self.x ** 2 + self.y ** 2)

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
    if b.norm2() < c.norm2():
      return Clockwise.abc
    return Clockwise.otherwise


def parse_pointstr(s):
  # parse '1/2,1/2'
  [a,b] = s.split(",")
  return Point(a,b)

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
    return p + d * Fraction(2)

  def intersect_LL(l1, l2):# 直線同士の交叉
    a = abs(Point.cross(l1.p2 - l1.p1, l2.p2 - l2.p1)) > EPS
    b = abs(Point.cross(l1.p2 - l1.p1, l2.p1 - l1.p1)) < EPS
    return a or b

  def intersect_LS(l, s):# 直線と線分の交叉
    a = Point.cross(l.p2 - l.p1, s.p1 - l.p1)
    b = Point.cross(l.p2 - l.p1, s.p2 - l.p1)
    return a * b < EPS

  def intersect_LP(l, p):# 直線上に点があるか
    return abs(Point.cross(l.p2 - p, l.p1 - p)) < EPS

  def intersect_SS(s, t):# 線分同士の交叉
    a = Point.ccw(s.p1, s.p2, t.p1) * Point.ccw(s.p1, s.p2, t.p2) <= 0
    b = Point.ccw(t.p1, t.p2, s.p1) * Point.ccw(t.p1, t.p2, s.p2) <= 0
    return a and b

  def intersect_SP(s, p):# 線分上に点があるか
    a = abs(s.p1 - p)
    b = abs(s.p2 - p)
    c = abs(s.p2 - s.p1)
    return a + b - c < EPS # 三角不等式

  def cross_point(l, m):# 直線の交点
    a = Point.dot(l.p2 - l.p1, m.p2 - m.p1)
    b = Point.dot(l.p2 - l.p1, l.p2 - m.p1)
    if abs(a) < EPS and abs(b) < EPS:
      return m.p1
    if abs(a) < EPS:
      assert False
    return m.p1 + b / a * (m.p2 - m.p1)

# 目的とする形
class Target:
  def __init__(self, filename):
    self.vs = []
    try:
      with open(filename) as f:
        np = int(f.readline())
        for p in range(np):
          nv = int(f.readline())
          for v in range(nv):
            self.vs.append(parse_pointstr(f.readline()))
    except Exception:
      print("error...")
      raise
    self.vs = self.take_convexhull()
    self.shift = Point(min(self.vs, key=lambda v: v.x).x,
                       min(self.vs, key=lambda v: v.y).y)
    #print(self.vs)
    # 平行移動
    self.vs = list(map(lambda v: v-self.shift, self.vs))


  def take_convexhull(self):
    # 頂点リストccwへ変換
    ln = len(self.vs)
    svs = sorted(self.vs, key=lambda p:p.x)
    nvs = [None for _ in range(ln*2)]
    i=0; k=0
    while i<ln:
      while k>1 and Point.ccw(nvs[k-2],nvs[k-1],svs[i])<=0:
        k-=1
      nvs[k] = svs[i]; i+=1; k+=1;
    i=ln-2; t=k+1
    while i>=0:
      while k>=t and Point.ccw(nvs[k-2],nvs[k-1],svs[i])<=0:
        k-=1
      nvs[k] = svs[i]; i-=1; k+=1;
    return nvs[:k-1]

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

  def solve(self, target):
    self.dv = list(map(lambda v: v+target.shift, self.dv))

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

  def intersect_LF(self, line, facet_id):# 直線とfacetが交叉するか
    facet = self.fs[facet_id]
    for i in range(len(facet)):
      edge = Line(self.dv(facet[i]), self.dv(facet[(i + 1) % len(facet)]))
      if Line.intersect_LS(line, edge):
        return True
    return False

  def fold(self, line, ccw_dir):
    assert(ccw_dir == Clockwise.ccw or ccw_dir == Clockwise.clockwise)
    new_sv = copy.deepcopy(self.sv)
    new_dv = copy.deepcopy(self.dv)
    new_fs = []

    # 新たに追加された点
    added_v = {}
    for f_id in range(len(self.fs)):
      if not Origami.intersect_LF(line, f_id):
        new_fs.append(self.fs[f_id])
        continue

      # 折線と交差するfacetについて
      facet = self.fs[facet_id]
      d_edges = []
      for i in range(len(facet)):
        edge = Line(self.dv(facet[i]), self.dv(facet[(i + 1) % len(facet)]))
        if Line.intersect_LS(line, edge):
          d_edges.append(edge)

      assert(len(d_edges) == 2)

      new_p1 = Line.cross_point(line, d_edges[0])
      new_p2 = Line.cross_point(line, d_edges[1])

      new_p1_src = None # TODO: new_p1の初期状態での座標
      new_p2_src = None # TODO: new_p2の初期状態での座標

      if new_p1 in added_v:
        new_id1 = added_v[new_p1]
      else:
        new_id1 = len(new_sv)
        new_sv.append(new_p1_src)
        new_dv.append(new_p1)
        added_v[new_p1] = new_id1

      if new_p2 in added_v:
        new_id2 = added_v[new_p2]
      else:
        new_id2 = len(new_sv)
        new_sv.append(new_p2_src)
        new_dv.append(new_p2)
        added_v[new_p2] = new_id2

      # facetの頂点を折り返しによって動くかどうかで分類
      fix_vs = []
      move_vs = []
      for v_id in facet:
        v = self.ds[v_id]
        if ccw_dir == Point.ccw(line.p1, line.p2, v):
          move_vs.append(v_id) # 折り返しで移動する頂点
        else:
          fix_vs.append(v_id)  # 折り返しで移動しない頂点

      # 移動する頂点達のdstを変更
      for v_id in move_vs:
        new_dvs[v_id] = line.lin_sym(self.dv[v_id])

      new_fs.append(fix_vs  + [n_v_id1, n_v_id2])
      new_fs.append(move_vs + [n_v_id1, n_v_id2])

    # TODO new_fsの中身を反時計周りになるようにソート (convex full?)
    new_fs = list(map(facet_sort, new_fs))

    self.sv = new_sv
    self.dv = new_dv
    self.fs = new_fs


def main():
  origami = Origami()
  target = Target("problems/problem_101.in")
  origami.solve(target)
  print(origami.to_s())

if __name__ == '__main__':
  main()
