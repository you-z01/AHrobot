
import torch
import torch.nn as nn
from models.common import Conv, Bottleneck

__all__ = ['ECAAttention', 'C3ECA', 'ECA']


class ECA(nn.Module):
    def __init__(self, c1, c2, k_size=3):
        super(ECA, self).__init__()
        self.avg_pool = nn.AdaptiveAvgPool2d(1)
        self.conv = nn.Conv1d(
            1, 1, kernel_size=k_size, padding=(k_size - 1) // 2, bias=False
        )
        self.sigmoid = nn.Sigmoid()

    def forward(self, x):
        y = self.avg_pool(x)
        y = self.conv(y.squeeze(-1).transpose(-1, -2)).transpose(-1, -2).unsqueeze(-1)
        y = self.sigmoid(y)
        return x * y.expand_as(x)



class ECAAttention(nn.Module):
    def __init__(self,  channel, kernel_size=5, stride=1, padding=1, gamma=2, b=1):
        super(ECAAttention, self).__init__()
        # k = int(abs((math.log(in_channel, 2) + b) / gamma))
        #kernel_size = k if k % 2 else k + 1
        padding = kernel_size // 2
        self.pool = nn.AdaptiveAvgPool2d(1)
        self.conv = nn.Sequential(
            nn.Conv1d(in_channels=1, out_channels=1, kernel_size=kernel_size,stride=stride, padding=padding, bias=False),
            nn.Sigmoid()
        )

    def forward(self, x):
        out = self.pool(x)
        out = out.view(x.size(0), 1, x.size(1))
        out = self.conv(out)
        out = out.view(x.size(0), x.size(1), 1, 1)
        return out * x

class C3ECA(nn.Module):
    def __init__(self, c1, c2, n=1, shortcut=True, g=1,
                 e=0.5):  # ch_in, ch_out, number, shortcut, groups, expansion #iscyy

        super(C3ECA, self).__init__()
        c_ = int(c2 * e)  # hidden channels
        self.eca = ECAAttention(2 * c_)
        self.cv1 = Conv(c1, c_, 1, 1)
        self.cv2 = Conv(c1, c_, 1, 1)
        self.cv3 = Conv(2 * c_, c2, 1)  # act=FReLU(c2)
        # self.m = nn.Sequential(*[CB2d(c_) for _ in range(n)])
        self.m = nn.Sequential(*[Bottleneck(c_, c_, shortcut, g, e=1.0) for _ in range(n)])

    def forward(self, x):
        out = torch.cat((self.m(self.cv1(self.eca(x))), self.cv2(self.eca(x))), dim=1)
        out = self.cv3(out)
        return out
if __name__ == '__main__':
    input = torch.randn(512, 512, 7, 7)
    pna = C3ECA(512, 512)
    output = pna(input)
    print(output.shape)
