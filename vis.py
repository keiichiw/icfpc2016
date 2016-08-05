#!/usr/bin/python3
# -*- encoding: utf-8 -*-
import sys
import math
import numpy as np
import pylab as pl
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
import argparse

def frac2num(s):
  x = s.find("/")
  if x == -1:
    return int(s)
  else:
    a = int(s[:x])
    b = int(s[x + 1:])
    return a / b

def parse_point(s):
  # print(s)
  [a, b] = s.split(",")
  return (frac2num(a), frac2num(b))

def read_input(infile):
  try:
    with open(infile, "r") as f:
      p = int(f.readline())
      assert(p == 1) # TODO: 1以外来る?
      vnum = int(f.readline())
      vs = []
      for i in range(vnum):
        l = f.readline()
        vs.append(parse_point(l))
      line_num = int(f.readline())
      lines = []
      for i in range(line_num):
        [p1, p2] = f.readline().split(" ")
        lines.append((parse_point(p1), parse_point(p2)))
      return {"vs": vs, "polygons": [vs], "lines": lines}
  except Exception:
    print("Input Error:" + sys.exc_info()[0])
    raise

def read_solution(infile):
  try:
    with open(infile, "r") as f:
      vnum = int(f.readline())
      vs = []
      for i in range(vnum):
        l = f.readline()
        vs.append(parse_point(l))
      pnum = int(f.readline())
      ps = []
      for i in range(pnum):
        idxs = [int(x) for x in f.readline().split()]
        ps.append(idxs[1:])
      mps = []
      for i in range(vnum):
        l = f.readline()
        mps.append(parse_point(l))
      return {"vs": vs, "polygons": ps, "mappings": mps}
  except Exception:
    print("Input Error:" + sys.exc_info()[0])
    raise

def plot_input(data, filename):
  plt.clf()
  # http://stackoverflow.com/a/21357666
  lc = mc.LineCollection(data["lines"], linewidths=2)
  fig, ax = pl.subplots()
  ax.set_aspect('equal')
  (c_x, c_y) = (0, 0)
  for [(x1, y1), (x2, y2)] in data["lines"]:
    c_x += x1 + x2
    c_y += y1 + y2
  c_x /= len(data["lines"]) * 2.0
  c_y /= len(data["lines"]) * 2.0
  pl.axis([-1 + c_x, 1 + c_x, -1 + c_y, 1 + c_y])
  pl.grid(True)
  ax.add_collection(lc)
  ax.margins(0.1)
  #plt.savefig(filename[0:-3]+".png")
  plt.show()

def plot_solution(data, filename):
  plt.clf()
  fig = plt.figure()
  # 展開図
  subplt = fig.add_subplot(2, 2, 1,aspect='equal')
  vs = data["vs"]
  lines1 = []
  for poly in data["polygons"]:
    poly.append(poly[0])
    for i in range(len(poly) - 1):
      lines1.append((vs[poly[i]], vs[poly[i + 1]]))
  lc = mc.LineCollection(lines1, linewidths=2)
  subplt.add_collection(lc)
  subplt.autoscale_view(True,True,True)
  pl.axis([-0.5, 1.5, -0.5, 1.5])
  pl.grid(True)
  plt.plot()

  # シルエット
  subplt1 = fig.add_subplot(2, 2, 2,aspect='equal')
  vs = data["mappings"]
  lines1 = []
  for poly in data["polygons"]:
    poly.append(poly[0])
    for i in range(len(poly) - 1):
      lines1.append((vs[poly[i]], vs[poly[i + 1]]))
  lc = mc.LineCollection(lines1, linewidths=2)
  subplt1.add_collection(lc)
  subplt1.autoscale_view(True,True,True)
  pl.axis([-0.5, 1.5, -0.5, 1.5])
  pl.grid(True)
  plt.plot()
  #pl.savefig(filename[0:-3]+".png")
  pl.show()

def main():
  # for i in range(101):
  #   filename = "./problems/problem_"+str(i+1)+".in"
  #   print(filename)
  #   try:
  #     data = read_input(filename)
  #     plot_input(data,filename)
  #   except Exception:
  #     pass
  # return

  parser = argparse.ArgumentParser()
  parser.add_argument('mode', choices=['inp', 'sol'], help="'input' or 'sol'")
  parser.add_argument("-i", "--input", type=str, required=True, help="file name (e.g. test.in)")
  args = parser.parse_args()
  infile = args.input
  if args.mode == "inp":
    data = read_input(infile)
    plot_input(data, infile)
  else:
    data = read_solution(infile)
    plot_solution(data, infile)

if __name__ == '__main__':
  main()
