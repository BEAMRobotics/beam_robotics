import keras
import keras.backend as K

# define function to calculate percentage of Delam labels
def perDelam(y_true, y_pred):
    true_delam = K.cast(K.sum(y_true), 'float64')
    pred_delam = K.cast(K.sum(K.argmax(y_pred, axis=-1)), 'float64')
    percent = pred_delam / true_delam * 100
    return percent

def predDelam(y_true, y_pred):
    pred_delam = K.cast(K.sum(K.argmax(y_pred, axis=-1)), 'float64')
    return pred_delam

def realDelam(y_true, y_pred):
    true_delam = K.cast(K.sum(y_true), 'float64')
    return true_delam

def customRescale(x):
    y = (x / (255/2) - 1.)
    return y

def iou_loss(true,pred):  #this can be used as a loss if you make it negative
    pred = K.cast(K.argmax(pred, axis=-1), K.floatx())
    pred = K.expand_dims(pred, 3)
    true = K.cast(true, K.floatx())
    intersection = true * pred
    notTrue = 1 - true
    union = true + (notTrue * pred)

    return (K.sum(intersection) + K.epsilon()) / (K.sum(union) + K.epsilon())

def mIOU(true,pred):
    intersection = true * pred
    notTrue = 1 - true
    union = true + (notTrue * pred)

    return (sum(sum(intersection)) + K.epsilon()) / (sum(sum(union)) + K.epsilon())

def zeroPad(imageIn, desiredSize, RGB):
    w, h, _ = imageIn.shape
    ratio = desiredSize / np.max([w,h])
    print(ratio)
    resized = cv2.resize(imageIn, (int(ratio*h),int(ratio*w)))
    pad_x = int(desiredSize - resized.shape[0])
    print(pad_x)
    if RGB:
        resized2 = np.pad(resized,((0,pad_x),(0,0),(0,0)),mode='constant')
    else:
        resized2 = np.pad(resized,((0,pad_x),(0,0),(0,0)),mode='constant')

    return resized2, pad_x
