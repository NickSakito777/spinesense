import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
TOOLS=Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0,str(TOOLS))
import review_taps as review


def sample(ms,sensor,x=0):
    return f'{ms} {sensor} label 0x10 {x} 0 1000 0 0 0 1 0 0 0\n'


class ReviewTests(unittest.TestCase):
    def parse(self,content):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'tap.log'; p.write_text(content)
            return review.parse_taps(p)

    def test_parses_all_five_magnitudes(self):
        traces,stats=self.parse(''.join(sample(t,f'IMU{i}',750) for t in (100,110) for i in range(5)))
        self.assertFalse(stats['problems'])
        self.assertEqual(traces['IMU0'][0],(100,1250))
        page=review.render(traces,stats,'<injected>')
        self.assertIn('&lt;injected&gt;',page)
        self.assertEqual(page.count('<svg '),5)

    def test_empty_blocks(self):
        traces,stats=self.parse(''); self.assertIn('Empty tap log.',stats['problems'])
        self.assertNotIn('<svg ',review.render(traces,stats,'empty'))

    def test_reset_suppresses_misleading_joined_plot(self):
        traces,stats=self.parse(sample(100,'IMU0')+sample(1,'IMU0'))
        self.assertEqual(stats['backward_jumps'],1)
        self.assertNotIn('<svg ',review.render(traces,stats,'reset'))

    def test_invalid_and_partial_explicit(self):
        traces,stats=self.parse(sample(100,'IMU0','nan')+sample(110,'IMU0').rstrip())
        self.assertEqual(stats['invalid_samples']['IMU0'],2)
        self.assertEqual(stats['partial_lines'],1)

    def test_extrema_downsample_keeps_impulses_and_order(self):
        points=[(i,1) for i in range(10000)]
        points[251]=(251,100); points[252]=(252,-100)
        reduced=review.downsample_extrema(points)
        self.assertIn(points[251],reduced); self.assertIn(points[252],reduced)
        self.assertLessEqual(len(reduced),2400)
        self.assertEqual(reduced,sorted(reduced))
        self.assertEqual(reduced[0],points[0]); self.assertEqual(reduced[-1],points[-1])

if __name__=='__main__': unittest.main()
