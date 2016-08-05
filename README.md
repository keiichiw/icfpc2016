#ICFPC16
* http://icfpc2016.blogspot.jp/

# Visualizer
入力・解を可視化する。

```
$ ./vis.py [inp | sol] -i <input file>
```

例:

入力
```
$ ./vis.py inp -i ./test/inp.in
```

解
```
$ ./vis.py sol -i ./test/sol.in
```

# 問題のダウンロード

```
$ ./call_api.py download
```

追加された問題のみダウンロードしてくる
ダウンロードには、[問題数]秒かかる (API制限のため)
