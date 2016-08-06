#!/usr/bin/python3
# -*- encoding: utf-8 -*-

import call_api
import main

import time
import argparse

def solve_submit(maxid):
    for i in range(1,maxid+1):
        try:
            origami = main.Origami()
            target = main.Target("problems/problem_{}.in".format(i))
            origami.solve(target)
            res = call_api.api_submit_sol(i, origami.to_s())
            print(res)
            time.sleep(1)
        except Exception:
            print("error in solving prob_{}".format(i))
            pass

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--maxid", type=int, required=True, help="maximum id for submition")
    args = parser.parse_args()
    solve_submit(args.maxid)
