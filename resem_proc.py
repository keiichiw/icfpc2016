#!/usr/bin/python3
# -*- encoding: utf-8 -*-

def main():
  data = []
  with open("./resem_data.txt", "r") as f:
    for s in f.readlines():
      s = s[8:]
      p_id = int(s[:s.find("'")])
      resem = float(s[s.find(":")+2:])
      data.append((p_id, resem))
  data = list(set(data))
  data.sort(key=lambda x:(x[1], x[0]))
  for (p_id, resem) in data:
    print("{}, {}".format(p_id, resem))

  print(len(data))
if __name__ == "__main__":
  main()
