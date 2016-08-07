#!/usr/bin/python3
# -*- encoding: utf-8 -*-
import time
import sys
import copy
from enum import IntEnum, Enum
import fractions
import math
from fractions import Fraction
import random

random.seed(0)

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

def is_square(q):
  n = q.numerator
  d = q.denominator
  return int(math.sqrt(n))**2==n and int(math.sqrt(d))**2==d
def square(q):
  n = q.numerator
  d = q.denominator
  return Fraction(int(math.sqrt(n)), int(math.sqrt(d)))


class Line:
  def __init__(self, p1, p2):
    self.p1 = p1
    self.p2 = p2

  def __repr__(self):
    return "({}, {})".format(self.p1, self.p2)

  def ccw(self, p):
    return Point.ccw(self.p1, self.p2, p)

  def lin_sym(self, point):
    a, b = self.p1.x, self.p1.y
    c, d = self.p2.x, self.p2.y
    p, q = point.x, point.y

    if a == c:
      x = 2 * a - p
      y = q
    elif b == d:
      x = p
      y = 2 * b - q
    else:
      l = (a - c) * (b - d) / ((b - d) ** 2 + (a - c) ** 2)
      r = q + (((c - a) / (d - b)) * p) - (b - ((d - b) / (c - a)) * a)
      s = l * r  # 交点のx座標
      x = 2 * s - p
      y = (-(a - c) / (b - d)) * x + q + ((a - c) / (b - d)) * p
    return Point(x, y)

  def intersect_LL(l1, l2):# 直線同士の交叉
    a = abs(Point.cross(l1.p2 - l1.p1, l2.p2 - l2.p1)) != 0
    b = abs(Point.cross(l1.p2 - l1.p1, l2.p1 - l1.p1)) == 0
    return a or b

  def intersect_LS(l, s):# 直線と線分の交叉 onlineのときもTrueを返す
    a = Point.cross(l.p2 - l.p1, s.p1 - l.p1)
    b = Point.cross(l.p2 - l.p1, s.p2 - l.p1)
    return a * b <= 0

  def intersect_LP(l, p):# 直線上に点があるか
    return abs(Point.cross(l.p2 - p, l.p1 - p)) == 0

  def intersect_SS(s, t):# 線分同士の交叉
    a = Point.ccw(s.p1, s.p2, t.p1) * Point.ccw(s.p1, s.p2, t.p2) <= 0
    b = Point.ccw(t.p1, t.p2, s.p1) * Point.ccw(t.p1, t.p2, s.p2) <= 0
    return a and b

  def intersect_SP(s, p):# 線分上に点があるか
    a = abs(s.p1 - p)
    b = abs(s.p2 - p)
    c = abs(s.p2 - s.p1)
    return a + b - c == 0 # 三角不等式

  def cross_point(l, m):# 直線の交点
    a = Point.cross(l.p2 - l.p1, m.p2 - m.p1)
    b = Point.cross(l.p2 - l.p1, l.p2 - m.p1)
    if abs(a) == 0 and abs(b) == 0:
      return m.p1
    if abs(a) == 0:
      assert False
    ret = m.p1 + (m.p2 - m.p1) * (b / a)

    return ret

# 目的とする形
class Target:
  def __init__(self, src):
    self.vs = []
    try:
      if isinstance(src, str):
        filename = src
        with open(filename) as f:
          np = int(f.readline())
          for p in range(np):
            nv = int(f.readline())
            for v in range(nv):
              self.vs.append(parse_pointstr(f.readline()))
      elif isinstance(src, list):
        self.vs = copy.deepcopy(src)
      else:
        assert(False)
    except Exception:
      print("error...")
      raise
    # 凸包
    self.vs = self.take_convexhull()
    # 回転
    (self.r_center, self.r_sin, self.r_cos) = self.find_rotate()
    self.rotate(self.r_center, -self.r_sin, self.r_cos)
    # 移動
    self.shift = Point(min(self.vs, key=lambda v: v.x).x,
                       min(self.vs, key=lambda v: v.y).y)
    self.vs = list(map(lambda v: v-self.shift, self.vs))

  def take_convexhull(self):
    # 頂点リストccwへ変換
    ln = len(self.vs)
    svs = sorted(self.vs, key=lambda p:(p.x, p.y))
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

  def find_rotate(self):
    # 最長有理数辺を軸に合わせる
    # 返り値は逆変換
    vnum = len(self.vs)
    p_id = -1
    maxline = Point(0,0)
    maxc = 0
    for i in range(vnum):
      p1 = self.vs[i]
      p2 = self.vs[(i+1) % vnum]
      line = p2-p1
      c2 = line.x**2 + line.y**2
      if not is_square(c2):
        continue
      c = square(c2)
      if c > maxc:
        p_id = i
        maxline = line
        maxc = c
    if p_id < 0:
      return (Point(0,0), 0, 1)

    sin = maxline.y / maxc
    cos = maxline.x / maxc
    return (self.vs[p_id], sin, cos)

  def rotate(self, center, s, c):
    # center:Point, s,c*Fraction
    assert(s**2+c**2==1)
    def sub_rotate(v):
      nx = (v.x-center.x)*c-(v.y-center.y)*s + center.x
      ny = (v.x-center.x)*s+(v.y-center.y)*c + center.y
      return Point(nx,ny)
    self.vs = list(map(sub_rotate, self.vs))

  def calc_area(self):
    area = Fraction(0)
    vsize = len(self.vs)
    for i in range(vsize):
      area += Point.cross(self.vs[i], self.vs[(i + 1) % vsize])
    return area

  def get_rect(self):
    mx_x, mn_x = self.vs[0].x, self.vs[0].x
    mx_y, mn_y = self.vs[0].y, self.vs[0].y
    for v in self.vs:
      mx_x = max(mx_x, v.x)
      mn_x = min(mn_x, v.x)
      mx_y = max(mx_y, v.y)
      mn_y = min(mn_y, v.y)

    p1 = Point(mn_x, mn_y)
    p2 = Point(mx_x, mn_y)
    p3 = Point(mx_x, mx_x)
    p4 = Point(mn_x, mx_y)

    return [Line(p1, p2), Line(p2, p3), Line(p3, p4), Line(p4, p1)]

class Origami:
  def __init__(self, filename=None):
    if filename == None:
      self.sv = [Point(0, 0), Point(1, 0),
                 Point(1, 1), Point(0, 1)]
      self.dv = [Point(0, 0), Point(1, 0),
                 Point(1, 1), Point(0, 1)]
      self.fs = [[0, 1, 2, 3]]
    else:
      try:
        with open(filename) as f:
          vnum = int(f.readline())
          self.sv = [None for _ in range(vnum)]
          for i in range (vnum):
            self.sv[i] = parse_pointstr(f.readline())
          fnum = int(f.readline())
          self.fs = [None for _ in range(fnum)]
          for i in range (fnum):
            idxs = f.readline().split()[1:]
            self.fs[i] = list(map(lambda s: int(s), idxs))
          self.dv = [None for _ in range(vnum)]
          for i in range (vnum):
            self.dv[i] = parse_pointstr(f.readline())
      except Exception:
        print("hey!")
        assert False
    assert (len(self.sv) == len(self.dv))

  def copy(self, other):
    self.sv = copy.deepcopy(other.sv)
    self.dv = copy.deepcopy(other.dv)
    self.fs = copy.deepcopy(other.fs)

  def solve(self, target):
    # 凸包生成
    self.greedy(target)
    # ターゲットに合わせる
    self.shift(target.shift)
    self.rotate(target.r_center, target.r_sin, target.r_cos)

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

  def sol_size(self, shift):
    origami = Origami()
    origami.copy(self)
    origami.shift(shift)
    return len("".join(origami.to_s().split()))

  def eval_fold(self, edge, shift):#折りの良さの評価関数
    # 折った後のシルエットの凸包の面積が小さい方が良い
    origami = Origami()
    origami.copy(self)

    before = Target(origami.dv)
    origami.fold(edge, Clockwise.clockwise)
    if origami.sol_size(shift) >= 5000:
      return -float("inf")
    after = Target(origami.dv)
    if before.calc_area() == after.calc_area():
      return -100000
    return 10
    return -after.calc_area()

  def greedy(self, target):
    v_size = len(target.vs)
    updated = True
    cnt = 0

    e_list = [Line(target.vs[i], target.vs[(i + 1) % v_size]) for i in range(v_size)]
    #e_list =  target.get_rect()

    start_time = time.time()
    while updated and cnt < 30:
      now_time = time.time()
      if now_time - start_time > 7:
        print("timeup")
        break
      updated = False
      max_ev = -float("inf")
      fold_edges = []
      v_list = self.dv
      for edge in e_list:
        for v in v_list:
          if edge.ccw(v) == Clockwise.clockwise:
            ev = self.eval_fold(edge, target.shift)
            if max_ev < ev:
              max_ev = ev
              fold_edges = [edge]
            elif len(fold_edges) > 0 and max_ev == ev:
              fold_edges.append(edge)

      if len(fold_edges) > 0:
        cnt += 1
        idx = random.randint(0, len(fold_edges) - 1)
        print("Fold {}: {}".format(cnt, fold_edges[idx]))
        self.fold(fold_edges[idx], Clockwise.clockwise)
        updated = True

    assert(self.sol_size(target.shift) < 5000)

  def rotate(self, center, s, c):
    # center:Point, s,c*Fraction
    assert(s**2+c**2==1)
    def sub_rotate(v):
      nx = (v.x-center.x)*c-(v.y-center.y)*s + center.x
      ny = (v.x-center.x)*s+(v.y-center.y)*c + center.y
      return Point(nx,ny)
    self.dv = list(map(sub_rotate, self.dv))

  def shift(self, move):
    def sub_shift(v):
      return Point(v.x+move.x, v.y+move.y)
    self.dv = list(map(sub_shift, self.dv))

  def intersect_LF(self, line, facet_id):# 直線とfacetが交叉するか
    facet = self.fs[facet_id]
    fsize = len(facet)
    for i in range(fsize):
      edge = Line(self.dv[facet[i]], self.dv[facet[(i + 1) % fsize]])
      if Line.intersect_LS(line, edge):
        return True
    return False

  def orig_position(src_edge, dst_edge, dst_point):
    (a1, b1) = (dst_edge.p1.x, dst_edge.p1.y)
    (c1, d1) = (dst_edge.p2.x, dst_edge.p2.y)

    (a2, b2) = (src_edge.p1.x, src_edge.p1.y)
    (c2, d2) = (src_edge.p2.x, src_edge.p2.y)

    (s1, t1) = (dst_point.x, dst_point.y)

    if a1 != c1:
      p = (c1 - s1) / (c1 - a1)
      q = (a1 - s1) / (c1 - a1)
    else:
      p = (d1 - t1) / (d1 - b1)
      q = (b1 - t1) / (d1 - b1)

    s2 = p * a2 - q * c2
    t2 = p * b2 - q * d2

    return Point(s2, t2)

  def facet_sort(vs):# 凸包を求めることで、facetの頂点をソートする
    # vs は (id, point)のリスト
    # 頂点リストccwへ変換
    ln = len(vs)
    svs = sorted(vs, key=lambda pair:pair[1].x)
    nvs = [None for _ in range(ln*2)]
    i=0; k=0
    while i<ln:
      while k>1 and Point.ccw(nvs[k-2][1],nvs[k-1][1],svs[i][1])<=0:
        k-=1
      nvs[k] = svs[i]; i+=1; k+=1;
    i=ln-2; t=k+1
    while i>=0:
      while k>=t and Point.ccw(nvs[k-2][1],nvs[k-1][1],svs[i][1])<=0:
        k-=1
      nvs[k] = svs[i]; i-=1; k+=1;
    return nvs[:k-1]

  def fold(self, line, ccw_dir):
    assert(ccw_dir == Clockwise.ccw or ccw_dir == Clockwise.clockwise)
    new_sv = copy.deepcopy(self.sv)
    new_dv = copy.deepcopy(self.dv)
    new_fs = []

    # 新たに追加された点
    added_v_dict = {}
    for f_id in range(len(self.fs)):
      # 折線と交差するfacetについて
      facet = self.fs[f_id]

      ## end_pointsの計算
      if not self.intersect_LF(line, f_id):
        end_points = []
      else:
        d_edges = []
        f_size = len(facet)
        for i in range(f_size):
          v_id1 = facet[i]
          v_id2 = facet[(i + 1) % f_size]
          edge = Line(self.dv[v_id1], self.dv[v_id2])
          if Line.intersect_LS(line, edge):
            d_edges.append((v_id1, v_id2))

        end_points = []
        for (v_id1, v_id2) in d_edges:
          edge = Line(self.dv[v_id1], self.dv[v_id2])
          if Line.intersect_LP(line, edge.p1) and Line.intersect_LP(line, edge.p2):
            # 折れ線上に辺がある場合
            end_points.append(v_id1)
            end_points.append(v_id2)
          elif Line.intersect_LP(line, edge.p1):
            # 端点1が折れ線上にある
            end_points.append(v_id1)
          elif Line.intersect_LP(line, edge.p2):
            # 端点2が折れ線上にある
            end_points.append(v_id2)
          else:
            # 交点がある
            src_edge = Line(self.sv[v_id1], self.sv[v_id2])
            dst_edge = Line(self.dv[v_id1], self.dv[v_id2])

            cp = Line.cross_point(line, dst_edge)
            cp_orig = Origami.orig_position(src_edge, dst_edge, cp)
            key = (cp_orig.x, cp_orig.y)
            if key in added_v_dict:
              end_v_id = added_v_dict[key]
            else:
              # 新たな点の生成
              end_v_id = len(new_sv)
              new_sv.append(cp_orig)
              new_dv.append(cp)
              added_v_dict[key] = end_v_id

            end_points.append(end_v_id)

        end_points = list(set(end_points))

        assert(len(end_points) == 1 or len(end_points) == 2)
      ## end_pointsの計算 ここまで

      # facetの頂点を折り返しによって動くかどうかで分類
      fix_vs = []
      move_vs = []
      for v_id in facet:
        v = self.dv[v_id]
        if ccw_dir == Point.ccw(line.p1, line.p2, v):
          move_vs.append(v_id) # 折り返しで移動する頂点
        elif ccw_dir == -Point.ccw(line.p1, line.p2, v):
          fix_vs.append(v_id)  # 折り返しで移動しない頂点
        else:
          assert(v_id in end_points)
          continue

      # 移動する頂点達のdstを変更
      for v_id in move_vs:
        new_dv[v_id] = line.lin_sym(self.dv[v_id])

      fix_facet = fix_vs + end_points
      move_facet = move_vs + end_points

      if len(fix_facet) >= 3:
        new_fs.append(fix_facet)

      if len(move_facet) >= 3:
        new_fs.append(move_facet)


    ## facetに関するループここまで
    sorted_new_fs = []
    for facet in new_fs:
      p_list = [(v_id, new_dv[v_id]) for v_id in facet]
      sorted_f = list(map(lambda p:p[0], Origami.facet_sort(p_list)))
      sorted_new_fs.append(sorted_f)
    self.sv = new_sv
    self.dv = new_dv
    self.fs = sorted_new_fs

def solve_problem(p_id):
  origami = Origami()
  target = Target("./problems/problem_" + str(p_id) + ".in")
  origami.solve(target)
  #print(origami.to_s())
  print(origami.sol_size(Point(0, 0)))
  outfile = "./problems/solution_" + str(p_id) + ".out"
  with open(outfile, "w") as f:
    f.write(origami.to_s())

def solve_all():
  for i in range(1, 102):
    solve_problem(i)

def main():
  solve_problem(3560)
  return
  target = Target("problems/problem_100.in")
  print(target.vs)
  # origami = Origami("ownProbs/arch_36_39.in")
  # origami.shift(Point("-1/2","-1/4"))
  # origami.rotate(Point("0","0"),Fraction("7/25"),Fraction("24/25"))
  # origami.fold(Line(Point("1","0"), Point("1","1")), Clockwise.ccw)
  # origami.shift(Point("-2","0"))

  origami = Origami()
  origami.solve(target)
  print(origami.to_s())

if __name__ == '__main__':
  main()
