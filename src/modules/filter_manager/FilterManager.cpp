


// FilterManager.cpp 
// 
// 
#include "modules/filter_manager/FilterManager.h"
// #include "FilterManager.h"

FilterManager::FilterManager() : _activeFilter(&_mahony), _currentType(FilterType::MAHONY), 
                                 _pendingSwitch(false), _pendingType(FilterType::MAHONY) {}

void FilterManager::begin(float sampleFreqHz) {
    _mahony.begin(sampleFreqHz);
    _madgwick.begin(sampleFreqHz);
    _vibEstimator.configure(0.15f, 0.05f, 1.5f); // Настройка вибрационного монитора
    _activeFilter = &_mahony;
    _currentType = FilterType::MAHONY;
}

void FilterManager::enableAdaptive(bool enable) {
    _adaptiveEnabled = enable;
    _mahony.setAdaptiveEnabled(enable);
    _madgwick.setAdaptiveEnabled(enable);
}

void FilterManager::update(float dt, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    if (_pendingSwitch) {
        float w, x, y, z, roll, pitch, yaw;
        _activeFilter->getQuaternion(w, x, y, z);
        _activeFilter->getEuler(roll, pitch, yaw);
        if (_pendingType == FilterType::MAHONY) _mahony.setInitialState(roll, pitch, yaw);
        else _madgwick.setInitialState(roll, pitch, yaw);
        _currentType = _pendingType;
        _pendingSwitch = false;
    }
    
    // 🔑 АДАПТИВНАЯ ПОДСТРОЙКА ПЕРЕД КАЖДЫМ ШАГОМ
    float vibLevel = _vibEstimator.compute(ax, ay, az);
    _activeFilter->setAdaptiveParams(vibLevel);
    
    _activeFilter->update(dt, ax, ay, az, gx, gy, gz, mx, my, mz);
}

void FilterManager::getQuaternion(float &w, float &x, float &y, float &z) const { _activeFilter->getQuaternion(w, x, y, z); }
void FilterManager::getEuler(float &roll, float &pitch, float &yaw) const { _activeFilter->getEuler(roll, pitch, yaw); }
