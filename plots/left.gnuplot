a = 1157
b = .789
c = -0.355
d = -60
func(x) = a * (x + b) ** (c) + d
fit func(x) 'left.cut' via a, b, c, d
plot func(x), 'left.cut'
