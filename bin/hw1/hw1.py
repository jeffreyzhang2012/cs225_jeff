import redis
import matplotlib.pyplot as plt
import numpy as np

def hlp(x):
    return [float(i) for i in x]

num = 15000;

qs = []

r = redis.Redis()
i = 0
while i < num:
    if(r.get("sai2::cs225a::controller_running")=="1"):
        qs.append(hlp(r.get("sai2::cs225a::panda_robot::sensors::q")[1:-1].split(',')))
        # print(qs[i])
        i += 1
qs = np.array(qs)
# x = np.linspace(0,np.pi/2,num)
Q, (ax1, ax2, ax3) = plt.subplots(1, 3)
# Q.suptitle('Controller 1 (Q1b): kp = 400, kv = 55')
# Q.suptitle('Controller 2 (Q2b): kp = 400, kv = 65')
# Q.suptitle('Controller 3 (Q3b): kp = 400, kv = 61')
# Q.suptitle('Controller 4 (Q4jb): kp = 400, kv = 40')
Q.suptitle('Controller 4 (Q5): kp = 400, kv = 40')
qs =qs / np.pi * 180
ax1.plot(qs[:,0]);ax1.set_ylabel(r'$q^1_t(^\circ)$');ax1.set_xlabel(r'$t$');ax1.set_title(r'$q^1_{ss}$ = %.3f$^\circ$' %qs[-1,0])
ax2.plot(qs[:,2]);ax2.set_ylabel(r'$q^3_t(^\circ)$');ax2.set_xlabel(r'$t$');ax2.set_title(r'$q^3_{ss}$ = %.3f$^\circ$' %qs[-1,2])
ax3.plot(qs[:,3]);ax3.set_ylabel(r'$q^4_t(^\circ)$');ax3.set_xlabel(r'$t$');ax3.set_title(r'$q^4_{ss}$ = %.3f$^\circ$' %qs[-1,3])
ax1.ticklabel_format(useOffset=False)
ax2.ticklabel_format(useOffset=False)
ax3.ticklabel_format(useOffset=False)
plt.show(Q)
