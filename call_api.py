#!/usr/bin/python3
# -*- encoding: utf-8 -*-
import argparse
import json
import requests
import time
import os

def api_hello():
  response = requests.get(
    'http://2016sv.icfpcontest.org/api/hello',
    headers={'X-API-Key': '131-63b8c959fa9c6723a25ed0420536ee98'})
  return response.json()
def api_snapshot_list():
  response = requests.get(
    'http://2016sv.icfpcontest.org/api/snapshot/list',
    headers={'X-API-Key': '131-63b8c959fa9c6723a25ed0420536ee98'})
  return response.json()["snapshots"]

def api_bolb(h, text_format=False):
  response = requests.get(
    'http://2016sv.icfpcontest.org/api/blob/' + h,
    headers={'X-API-Key': '131-63b8c959fa9c6723a25ed0420536ee98'})
  if text_format:
    return response.text
  return response.json()


def get_problem(problem_id, problem_hash, filepath):
  r = api_bolb(problem_hash, text_format=True)
  print(filepath)
  with open(filepath, "w") as f:
    f.write(r)

def get_recent_status():
  snaps = api_snapshot_list()
  recent_time = 0
  recent_hash = None
  for snap in snaps:
    if recent_time < int(snap["snapshot_time"]):
      recent_time = int(snap["snapshot_time"])
      recent_hash = snap["snapshot_hash"]
  print("Recent hash: " + recent_hash)
  time.sleep(1)
  recent = api_bolb(recent_hash)
  return recent

def download_problems():
  problems = get_recent_status()["problems"]
  for p in problems:
    p_id = p['problem_id']
    p_hash = p['problem_spec_hash']
    filename = "problem_{}.in".format(p_id)
    cur_path = os.path.dirname(os.path.realpath(__file__))
    pathname = cur_path + "/problems/" + filename
    if os.path.exists(pathname):
      print("Skip! Problem " + str(p_id))
    else:
      print("Downloading Problem " + str(p_id))
      time.sleep(1)
      txt = get_problem(p_id, p_hash, pathname)

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('mode', choices=['download', 'hello'], help="download")
  args = parser.parse_args()
  if args.mode == "hello":
    print(api_hello())
  elif args.mode == "download":
    download_problems()

if __name__ == '__main__':
  main()
