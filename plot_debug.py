
# ---- debug ----
t = np.arange(N+1)
tu = np.arange(N)

plt.figure(10)
plt.subplot(221)
plt.plot(t,opti.debug.value(xh),t,opti.debug.value(yh))
plt.legend(['xh','yh'])
plt.subplot(222)
plt.plot(t,opti.debug.value(tha1)/deg,t,opti.debug.value(tha2)/deg,t,opti.debug.value(tha3)/deg)
plt.legend(['tha1','tha2','tha3'])
plt.subplot(223)
plt.plot(tu,opti.debug.value(Mw1),tu,opti.debug.value(Mw2),tu,opti.debug.value(Mw3))
plt.legend(['Mw1','Mw2','Mw3'])
plt.subplot(224)
plt.plot(tu,opti.debug.value(Ma1),tu,opti.debug.value(Ma2),tu,opti.debug.value(Ma3))
plt.legend(['Ma1','Ma2','Ma3'])

# plt.figure(20)
# plt.spy(opti.debug.value(jacobian(opti.g, opti.x)))
# plt.figure(30)
# plt.spy(opti.debug.value(hessian(opti.f+dot(opti.lam_g, opti.g), opti.x)[0]))

plt.show()
