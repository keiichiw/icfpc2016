#!/usr/bin/python3
# -*- encoding: utf-8 -*-

import call_api
import origami
import os
import sys
import time
import argparse
from datetime import datetime as dt

CUR_DIR = os.path.dirname(os.path.realpath(__file__))

def get_problems():
    problem_dir = CUR_DIR + "/problems/"
    files = list(os.walk(problem_dir))[0][2]
    infiles = list(filter(lambda x:os.path.splitext(x)[1] == ".in", files))

    p_ids = []
    for f in infiles:
        try:
            assert(f[:8] == "problem_")
            assert(f[-3:] == ".in")
            p_id = int(f[8:-3])
            p_ids.append(p_id)
        except Exception:
            print("invalid file name: ", f)
            pass
    p_ids.sort()
    return p_ids

def get_submitted():
    submitted = []
    with open(CUR_DIR + "/submitted.txt", "r") as f:
        lns = f.readlines()
        for x in lns:
            submitted.append(int(x.rstrip()))

    solved = []

    with open(CUR_DIR + "/solved.txt", "r") as f:
        lns = f.readlines()
        for x in lns:
            solved.append(int(x.rstrip()))

    return (submitted, solved)

def write_submitted(submitted, solved):
    submitted = list(set(submitted))
    solved = list(set(solved))
    with open(CUR_DIR + "/submitted.txt", "w") as f:
        for p_id in submitted:
            print(p_id, file=f)

    with open(CUR_DIR + "/solved.txt", "w") as f:
        for p_id in solved:
            print(p_id, file=f)
    return (submitted, solved)

def write_log(result):
    tdatetime = dt.now()
    tstr = tdatetime.strftime('%m-%d-%H-%M-%S')
    logfile = CUR_DIR + "/problems/submit_{}.log".format(tstr)
    with open(logfile, "w") as f:
        for (p_id, res) in result:
            print(p_id, " : ", res, file=f)
    print("write log file : ", logfile)

def submit_problem(p_id):
    solfile = CUR_DIR + "/problems/solution_{}.out".format(p_id)
    with open(solfile, "r") as f:
        content = f.read()
        res = call_api.api_submit_sol(p_id, content)
    return res

def solve_submit_problem(p_id):
    try:
        origami.solve_problem(p_id)
    except Exception:
        print("error in solving prob_{}".format(p_id))
        raise

    try:
        ret = submit_problem(p_id)
    except Exception:
        print("error in submitting prob_{}".format(p_id))
        raise

    return ret

def submit_unsubmitted():
    p_list = get_problems()
    (submitted, solved) = get_submitted()
    result = []

    for p_id in p_list:
        try:
            if p_id in submitted:
                continue
            if p_id in solved:
                continue
            print("Problem {}".format(p_id))
            res = solve_submit_problem(p_id)
            if not res['ok']:
                print("NG response: Problem {}".format(p_id))
                print(res)
            else:
                submitted.append(p_id)
                resem = res['resemblance']
                if resem == 1.0:
                    solved.append(p_id)
                print("Problem {}' resemblance: {}".format(p_id, resem))
            result.append((p_id, res))
            time.sleep(3.7)
        except Exception:
            print("Error in submit_unsubmitted {}".format(p_id) + sys.exc_info()[0])
            pass
    submitted.sort()
    solved.sort()
    write_submitted(submitted, solved)
    write_log(result)
    print("end")

def solve_submit(maxid):
    for i in range(1,maxid+1):
        try:
            origami = origami.Origami()
            target = origami.Target("problems/problem_{}.in".format(i))
            origami.solve(target)
            res = call_api.api_submit_sol(i, origami.to_s())
            print(res)
            time.sleep(1)
        except Exception:
            print("error in solving prob_{}".format(i))
            pass

if __name__ == "__main__":
    submit_unsubmitted()
    #parser = argparse.ArgumentParser()
    #parser.add_argument("-m", "--maxid", type=int, required=True, help="maximum id for submition")
    #args = parser.parse_args()
    #solve_submit(args.maxid)
