/****************************************************************************
** Meta object code from reading C++ file 'CameraROS.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/find_object_2d/src/ros/CameraROS.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'CameraROS.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_CameraROS[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      45,   11,   10,   10, 0x05,

 // slots: signature, parameters, type, tag, flags
      98,   10,   10,   10, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_CameraROS[] = {
    "CameraROS\0\0frameId,stamp,depth,depthConstant\0"
    "rosDataReceived(std::string,ros::Time,cv::Mat,float)\0"
    "takeImage()\0"
};

void CameraROS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        CameraROS *_t = static_cast<CameraROS *>(_o);
        switch (_id) {
        case 0: _t->rosDataReceived((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const ros::Time(*)>(_a[2])),(*reinterpret_cast< const cv::Mat(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4]))); break;
        case 1: _t->takeImage(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData CameraROS::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject CameraROS::staticMetaObject = {
    { &find_object::Camera::staticMetaObject, qt_meta_stringdata_CameraROS,
      qt_meta_data_CameraROS, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &CameraROS::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *CameraROS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *CameraROS::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_CameraROS))
        return static_cast<void*>(const_cast< CameraROS*>(this));
    typedef find_object::Camera QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int CameraROS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef find_object::Camera QMocSuperClass;
    _id = QMocSuperClass::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void CameraROS::rosDataReceived(const std::string & _t1, const ros::Time & _t2, const cv::Mat & _t3, float _t4)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)), const_cast<void*>(reinterpret_cast<const void*>(&_t3)), const_cast<void*>(reinterpret_cast<const void*>(&_t4)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
