import numpy as np

def kfold_split(x,y,K,kcurr):
    # Extracting number of samples in the data
    N = len(x)

    # Specifying the number of samples in a fold
    Nfold = int(np.floor(N/K))

    ### <--- START OF YOUR CODE
    # Defining validation set indices
    idxVal_start = kcurr * Nfold
    idxVal_end = (kcurr + 1) * Nfold if kcurr < K - 1 else N  # Ensuring the last fold gets remaining data

    # Creating index masks for validation and training sets
    idxVal = np.arange(idxVal_start, idxVal_end)    
    idxPreVal = np.setdiff1d(np.arange(N), idxVal)  # Remaining data for training (pre-validation)
    
    #idxVal = 0
    #idxPreVal = 0

    ### END OF YOUR CODE --->

    x_preval = x[idxPreVal]
    y_preval = y[idxPreVal]

    x_val = x[idxVal]
    y_val = y[idxVal]

    return x_preval, y_preval, x_val, y_val
