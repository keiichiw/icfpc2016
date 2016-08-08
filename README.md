#ICFP Programming Contest 2016
* http://icfpc2016.blogspot.jp/

* チーム名: typo

## 方針
* 入力シルエットの凸包をとる
* 長さが有理数であるような辺があれば、そのうち最大のものがx軸と重なるように回転
* 入力シルエットの凸包ができるまで折る
  - 折る辺の選択は基本ランダム
  - solution size limitに達しても、resemblanceが極端に小さい場合は、折ることで全体の面積が小さくなるような辺から優先的に折るような方針も試してみる
