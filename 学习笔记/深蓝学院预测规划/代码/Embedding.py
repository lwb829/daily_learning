import torch.nn as nn

#corpus是语料库
corpus = "那是最美好的时代，那是最糟糕的时代；那是智慧的年头，那是愚昧的年头；那是信仰的时期，那是怀疑的时期；那是光明的季节，那是黑暗的季节；那是希望的春天，那是失望的冬天；我们拥有一切，我们一无所有；我们全都在直奔天堂，我们全都在直奔相反的方向一一简而言之，那时跟现在非常相像，某些最喧嚣的权威坚持要用形容词的最高级来形容它。说它好，是最高级的；说它不好，也是最高级的"

#建立语料库对应的字典
word_dict = {}
word_set = set()
for it in corpus:
    word_set.add(it)
index = 0
for it in word_set:
    word_dict[index] = it
    index = index + 1

#这我们想要embedding的句子
input = "失望之冬，希望之春"

#用索引表示句子
raw_input = []
for it in input:
    index_it = list(word_dict.keys())[list(word_dict.values()).index(it)]
    raw_input.append(index_it)

#得到raw_input为：[58, 66, 22, 62, 80, 41, 66, 22, 72]

#设置embedding的参数
embedding  = nn.Embedding(len(word_dict),3)

#打印最终embedding的结果
print(embedding(input))
