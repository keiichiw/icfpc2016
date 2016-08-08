#!/usr/bin/python3
# -*- encoding: utf-8 -*-

import call_api
import origami
import os
import sys
import time
import random
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

def get_own_problems():
    own = []
    with open(CUR_DIR + "/own.txt", "r") as f:
        lns = f.readlines()
        for x in lns:
            own.append(int(x.rstrip()))
    return own

def get_again_problems():
    l = []
    with open(CUR_DIR + "/again.txt", "r") as f:
        lns = f.readlines()
        for x in lns:
            l.append(int(x.rstrip()))
    l.sort()
    return l

def write_submitted(submitted, solved):
    submitted = list(set(submitted))
    solved = list(set(solved))
    submitted.sort()
    solved.sort()

    with open(CUR_DIR + "/submitted.txt", "w") as f:
        for p_id in submitted:
            print(p_id, file=f)

    with open(CUR_DIR + "/solved.txt", "w") as f:
        for p_id in solved:
            print(p_id, file=f)
    return (submitted, solved)

def write_again(again):
    again = list(set(again))
    again.sort()

    with open(CUR_DIR + "/again.txt", "w") as f:
        for p_id in again:
            print(p_id, file=f)

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

def solve_submit_problem(p_id, n_time, flg):
    try:
        if flg:
            origami.solve_problem(p_id, timelimit=120)
        else:
            origami.solve_problem(p_id, timelimit=30)
    except Exception:
        print("error in solving prob_{}".format(p_id))
        raise
    now = time.time()
    w_time = max(0, n_time - now)
    time.sleep(w_time)
    try:
        ret = submit_problem(p_id)
    except Exception:
        print("error in submitting prob_{}".format(p_id))
        raise
    n_time = time.time() + 3.7

    if ret['ok'] and ret['resemblance'] < 0.25:
        print("resem={}, so try again!".format(ret['resemblance']))
        try:
            if flg:
                origami.solve_problem(p_id, eval_flg=True, timelimit=120)
            else:
                origami.solve_problem(p_id, eval_flg=True, timelimit=45)
        except Exception:
            print("error in solving prob_{}".format(p_id))
            raise
        now = time.time()
        w_time = max(0, n_time - now)
        time.sleep(w_time)
        try:
            ret = submit_problem(p_id)
        except Exception:
            print("error in submitting prob_{}".format(p_id))
            raise
        print("new resem={}".format(ret['resemblance']))
    return ret, time.time()

def submit_problems(rand_flg=False, rand_num=0):
    p_list = get_problems()
    (submitted, solved) = get_submitted()
    again = get_again_problems()
    own = get_own_problems()
    result = []
    if rand_flg:
        p_list = again + random.sample(p_list, rand_num)
    n_time = time.time()
    for p_id in p_list:
        try:
            if p_id in solved:
                continue
            if p_id in own:
                continue
            if not rand_flg:
              if p_id in submitted:
                  continue
            print("Problem {}".format(p_id))
            res, p_time = solve_submit_problem(p_id, n_time, rand_flg)
            n_time = p_time + 3.7
            if not res['ok']:
                print("NG response: Problem {}".format(p_id))
                print(res)
            else:
                submitted.append(p_id)
                resem = res['resemblance']
                if resem == 1.0:
                    solved.append(p_id)
                print("Problem {}' resemblance: {}".format(p_id, resem))
            write_submitted(submitted, solved)
            if p_id in again:
                again.remove(p_id)
                write_again(again)
            result.append((p_id, res))
        except Exception:
            print("Error in submit_unsubmitted {}".format(p_id), sys.exc_info()[0])
            pass
    submitted.sort()
    solved.sort()
    write_submitted(submitted, solved)
    write_again(again)
    write_log(result)
    print("end")


def main():
    call_api.download_problems()
    while True:
        try:
            submit_problems()
            time.sleep(1)
            download_num = call_api.download_problems()
            cnt = 0
            while download_num == 0 and cnt < 20:
                print("random 30")
                submit_problems(rand_flg=True, rand_num=30)
                time.sleep(1)
                download_num = call_api.download_problems()
                cnt += 1
        except Exception:
            print("Error in main", sys.exc_info()[0], sys.exc_info()[1])
            pass

if __name__ == "__main__":
    main()

    #parser = argparse.ArgumentParser()
    #parser.add_argument("-m", "--maxid", type=int, required=True, help="maximum id for submition")
    #args = parser.parse_args()
    #solve_submit(args.maxid)
