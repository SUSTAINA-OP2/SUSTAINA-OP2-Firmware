#https://qiita.com/NNNiNiNNN/items/4fd5367f9ead6e5905a9
import numpy as np
import matplotlib.pyplot as plt
import argparse

def reg1dim(x, y):
    n = len(x)
    #a = ((np.dot(x, y) - y.sum() * x.sum() / n ) / ((x**2).sum() - x.sum()**2 / n))
    a = ((np.dot(x, y) - y.sum() * x.sum() / n ) / ((x**2).sum() - x.sum()**2 / n))
    b = (y.sum() - a * x.sum())/n

    return a, b

parser = argparse.ArgumentParser()

"""
parser.add_argument('offset', help='offset')
parser.add_argument('weight', help='weigt value', nargs='*')
parser.add_argument('weight_correct', help='correct_weight value', nargs='*')
"""
parser.add_argument('--offset', help='offset', required=True)
parser.add_argument('--weight', help='weigt value', nargs='*', required=True)
parser.add_argument('--weight_correct', help='correct_weight value', nargs='*', required=True)

args = parser.parse_args()


x = np.array(args.weight, dtype=float)
y = np.array(args.weight_correct, dtype=float)
print(f"{type(x)}")
print(f"{type(y)}")


a, b = reg1dim(x, y)

print(f"{a, b}")

plt.scatter(x, y, color="k")
plt.plot([0, x.max()], [b, a*x.max() + b])
plt.title(str(f"y = {a}*x + {b}"))
plt.show()

