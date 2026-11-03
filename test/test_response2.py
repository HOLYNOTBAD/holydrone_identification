"""用 SciPy 独立状态空间响应验证二阶预测和参数恢复。"""
import unittest
import numpy as np
from scipy.signal import StateSpace, lsim
from response2 import predict, fit_channel, score, identify_response


def independent(t, u, initial, wn, zeta, delay, gain, bias):
    shift = round(delay/(t[1]-t[0]))
    delayed = np.r_[np.full(shift,u[0]),u[:-shift]] if shift else u
    model = StateSpace([[0,1],[-wn*wn,-2*zeta*wn]], [[0],[wn*wn]], [[1,0]], [[0]])
    return lsim(model, gain*delayed+bias, t, X0=[initial,0], interp=False)[1]


class SecondOrderTest(unittest.TestCase):
    def test_prediction_against_state_space(self):
        t=np.arange(301)*.004
        u=np.where(t<.2,.1,np.where(t<.6,.3,-.2))
        for damping in [.3,1.,1.5]:
            params=(30.,damping,.024,1.1,.02)
            expected=independent(t,u,.08,*params)
            np.testing.assert_allclose(predict(t,u,.08,*params),expected,atol=1e-10)

    def test_fit_and_independent_holdout(self):
        truth=(25.,.3,.024,1.05,.01)
        def pulse(width):
            t=np.arange(301)*.004
            u=np.where(t<.2,0.,np.where(t<.2+width,.25,np.where(t<.2+2*width,-.25,0.)))
            return t,u,independent(t,u,.01,*truth)
        fit,boundary=fit_channel([pulse(.2),pulse(.35)],2)
        self.assertFalse(boundary)
        np.testing.assert_allclose(fit,truth,rtol=.01,atol=.001)
        self.assertLess(score([pulse(.28)],fit)['normalized_rmse'],.005)

    def test_same_data_rejected(self):
        import tempfile
        from pathlib import Path
        with tempfile.TemporaryDirectory() as root:
            a,b=Path(root)/'a.csv',Path(root)/'b.csv'
            a.write_text('identical'); b.write_text('identical')
            with self.assertRaises(ValueError): identify_response(a,b)


if __name__=='__main__': unittest.main()
