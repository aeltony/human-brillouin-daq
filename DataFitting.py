# By Amira, 02/2019, for fitting spectral data (Lorentzians)
import numpy as np
from lmfit import Model
from lmfit.models import LorentzianModel, ConstantModel
from scipy.optimize import curve_fit
from scipy.signal import find_peaks
from timeit import default_timer as timer
from scipy.ndimage.filters import gaussian_filter

#### Fit Brillouin spectrum,
# sline is the data (counts) for the pixels on the spectral line,
# ftol and xtol are fitting tolerances (adjust for speed vs. accuracy)
def fitSpectrum(sline, xtol=1e-6, ftol=1e-6, maxfev=500):
	weights = np.sqrt(sline)  # Weight data by SNR

	prominence = 0.4*np.amax(sline)
	# Find # of peaks
	peaks, pk_info = find_peaks(sline, prominence=prominence, width=2, height=100, rel_height=0.5)
	pk_wids = 0.5*pk_info['widths']
	pk_hts = np.pi*pk_wids*pk_info['peak_heights']

	# Remove false (noise) peak
	if len(peaks) == 3:
		idx = np.nanargmin(pk_hts)
		if idx == 0:
			peaks = peaks[1:]
			pk_wids = pk_wids[1:]
			pk_hts = pk_hts[1:]
		elif idx == 1:
			peaks = peaks[[0,2]]
			pk_wids = pk_wids[[0,2]]
			pk_hts = pk_hts[[0,2]]
		elif idx == 2:
			peaks = peaks[:-1]
			pk_wids = pk_wids[:-1]
			pk_hts = pk_hts[:-1]

	# Fit dual or quad Lorentzian peaks + constant background:
	if len(peaks) in [2, 4]:
		# Define model to fit spectrun to
		model = ConstantModel(prefix='bg_')
		for k in range(len(peaks)):
			model += LorentzianModel(prefix='pk'+str(k+1)+'_')

		# Initial guess for fitting
		pars = model.make_params()
		pars['bg_c'].set(100, min=0, max=2000)
		for k in range(len(peaks)):
			pars['pk'+str(k+1)+'_sigma'].set(pk_wids[k], min=0, max=len(sline))
			pars['pk'+str(k+1)+'_center'].set(peaks[k], min=0, max=len(sline))
			pars['pk'+str(k+1)+'_amplitude'].set(pk_hts[k], min=0, max=1e6)

		# Fit spectral line data to model
		px = np.linspace(0,len(sline)-1, len(sline))
		init = model.eval(pars,x=px)
		
		try:
			out = model.fit(sline, pars, x=px, weights=weights, fit_kws={'ftol':ftol,'xtol':xtol,'maxfev':maxfev})
			# print(out.fit_report(min_correl=0.5))
			fittedSpect = model.eval(out.params,x=px)

			# Average peak height (counts) and stepsances between peaks (pixels)
			if len(peaks)==2:
				interPeaksteps = np.zeros(2)
				interPeaksteps[0] = np.array([np.average(pk_info['peak_heights'])])
				interPeaksteps[1] = np.array([out.params['pk2_center'].value - out.params['pk1_center'].value])
			elif len(peaks)==4:
				interPeaksteps = np.zeros(3)
				interPeaksteps[0] = np.array([np.average(pk_info['peak_heights'])])
				interPeaksteps[1] = out.params['pk4_center'].value - out.params['pk1_center'].value
				interPeaksteps[2] = out.params['pk3_center'].value - out.params['pk2_center'].value
		except:
			print('[AndorDevice] Fitting spectrum failed')
			interPeaksteps = np.array([])
			fittedSpect = np.nan*np.ones(sline.shape)
	else:
		interPeaksteps = np.array([])
		fittedSpect = np.nan*np.ones(sline.shape)

	return (interPeaksteps, fittedSpect)

#### Segment Brillouin A-line (depth scan) into air / cornea / aq humor,
# steps is scan stepsance (depth), aline is Brillouin values over the scan depth,
# signal is peak Brillouin signal counts over the scan depth
def fitAline(steps, aline, signal):
	start = timer()

	nonanIdx = ~np.isnan(aline)
	aline = aline[nonanIdx]
	steps = steps[nonanIdx]
	signal = signal[nonanIdx]
	sigThresh = 0.45*np.mean(signal[signal>500])
	cleanIdx = (signal > sigThresh) & (aline > 4) & (aline < 8)

	if np.sum(cleanIdx==True) < 3:
		stromaBS = np.nan
		stromaIdx = []
		return (stromaBS, stromaIdx)

	if np.std(aline[cleanIdx]) < 0.15:
		stromaBS = np.mean(aline[cleanIdx])
		stromaIdx = steps[cleanIdx].astype(int)
		return (stromaBS, stromaIdx)

	aline = aline[cleanIdx]
	steps = steps[cleanIdx]
	signal = signal[cleanIdx]
	weights = np.sqrt(signal)

	blurred = gaussian_filter(aline, sigma=1)
	deriv = np.gradient(blurred,steps)
	edgeIdx = np.argmin(deriv)
	sortAline = np.argsort(blurred)
	if len(blurred) > 8:
		stromaGuess = np.mean(blurred[sortAline[-4:]])
		aqGuess = np.mean(blurred[sortAline[:4]])
		print('stromaGuess =', stromaGuess)
		print('aqGuess =', aqGuess)
	else:
		stromaGuess = np.amax(blurred, axis=0)
		aqGuess = np.amin(blurred, axis=0)
	widthGuess = (aqGuess - stromaGuess)/deriv[edgeIdx]
	endGuess = steps[edgeIdx]-0.5*widthGuess

	model = Model(alineShape)
	pars = model.make_params()
	pars['endStroma'].set(endGuess, min=steps[0], max=steps[-1])
	pars['stromaBS'].set(stromaGuess, min=5.4, max=6.)
	pars['width'].set(widthGuess, min=30., max=1000)
	pars['aqBS'].set(aqGuess, min=5.0, max=5.4)
	print('pars = ', pars)

	try:
		result = model.fit(aline, pars, x=steps, weights=weights, method='Nelder', fit_kws={'ftol':1e-6,'xtol':1e-6})
		print(result.fit_report())
		idxEnd = (np.abs(steps - result.params['endStroma'].value)).argmin()
		strIdx = (steps < steps[idxEnd]) & (abs(aline - stromaGuess) < 0.1)
	except:
		print('[ScanManager/run] Segmentation could not fit A-line')
		idxEnd = (np.abs(steps - endGuess)).argmin()
		strIdx = (steps < steps[idxEnd]) & (abs(aline - stromaGuess) < 0.1)

	if np.sum(strIdx==True) > 0:
		strPts = aline[strIdx]
	elif (np.sum(strIdx==True)==0) & (len(aline[:idxEnd])>0):
		strIdx = np.linspace(0,idxEnd-1,idxEnd)
		strPts = aline[:idxEnd]
	elif (np.sum(strIdx==True)==0) & (len(aline[:idxEnd])==0):
		strIdx = np.linspace(0,idxEnd-1,idxEnd)
		strPts = np.array([stromaGuess])

	try:
		stromaBS = np.nanmean(strPts)
	except:
		print('[ScanManager/run] Segmentation could not fit A-line')
		stromaBS = np.nan

	try:

		stromaIdx = steps[strIdx].astype(int)
	except:
		stromaIdx = []

	end = timer()
	print('[ScanManager/run] Segmentation time =', end-start)

	return (stromaBS, stromaIdx)


def alineShape(x, endStroma, stromaBS, width, aqBS):
    condlist = [ x < endStroma,
                (x >= endStroma) & (x < endStroma + width),
                x >= endStroma + width
                ]
    funclist = [lambda x: stromaBS,
                lambda x: x*(aqBS-stromaBS)/width + stromaBS - endStroma*(aqBS-stromaBS)/width,
                lambda x: aqBS
                ]
    return np.piecewise( x, condlist, funclist )


# OLD VERSION:
# def fitAline(steps, aline, signal):
# 	weights = np.sqrt(signal)  # Weight data by SNR
# 	# Get rid of NaNs and saturated points
# 	cleanIdx = ~np.isnan(aline)
# 	signal[~cleanIdx] = np.nan
# 	aline[~cleanIdx] = np.nan
# 	weights[~cleanIdx] = 0

# 	# Find signal spike corresponding to cornea surface
# 	idx = int(np.floor(0.5*len(signal)))
# 	idxStart = np.nanargmax(signal[:idx]) - 1
# 	if idxStart > len(signal)-3 or idxStart < 0:
# 		idxStart = 0
# 	startPos = steps[idxStart]

# 	stromaGuess = np.nanmean(aline[idxStart:idxStart+2])
# 	aqGuess = np.nanmean(aline[-3:])
# 	widthGuess = 0.2*steps[-1]

# 	model = Model(alineShape, nan_policy='omit')
# 	pars = model.make_params()
# 	pars['endStroma'].set(startPos+500., min=startPos, max=steps[-1])
# 	pars['stromaBS'].set(stromaGuess, min=5.45, max=7.)
# 	pars['width'].set(widthGuess, min=0., max=steps[-1])
# 	pars['aqBS'].set(aqGuess, min=5., max=5.45)

# 	print('pars = ', pars)

# 	try:
# 		result = model.fit(aline, pars, x=steps, weights=weights, nan_policy='omit')
# 		print(result.fit_report())
# 		idxEnd = (np.abs(steps - result.params['endStroma'].value)).argmin()
# 	except:
# 		print('[ScanManager/run] Segmentation failed.')
# 		idxEnd = idxStart + 3

# 	strPts = aline[idxStart:idxEnd]
# 	strPts = strPts[~np.isnan(strPts)] # Remove NaNs
# 	if len(strPts) == 0:
# 		stromaBS = np.nan
# 	else:
# 		stromaBS = np.nanmean(strPts)

# 	return (stromaBS, idxStart, idxEnd)