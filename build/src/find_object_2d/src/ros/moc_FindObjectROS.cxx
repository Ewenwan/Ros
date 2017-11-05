/****************************************************************************
** Meta object code from reading C++ file 'FindObjectROS.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/src/find_object_2d/src/ros/FindObjectROS.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'FindObjectROS.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_FindObjectROS[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      20,   15,   14,   14, 0x0a,
      90,   56,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_FindObjectROS[] = {
    "FindObjectROS\0\0info\0"
    "publish(find_object::DetectionInfo)\0"
    "frameId,stamp,depth,depthConstant\0"
    "setDepthData(std::string,ros::Time,cv::Mat,float)\0"
};

void FindObjectROS::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        FindObjectROS *_t = static_cast<FindObjectROS *>(_o);
        switch (_id) {
        case 0: _t->publish((*reinterpret_cast< const find_object::DetectionInfo(*)>(_a[1]))); break;
        case 1: _t->setDepthData((*reinterpret_cast< const std::string(*)>(_a[1])),(*reinterpret_cast< const ros::Time(*)>(_a[2])),(*reinterpret_cast< const cv::Mat(*)>(_a[3])),(*reinterpret_cast< float(*)>(_a[4]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData FindObjectROS::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject FindObjectROS::staticMetaObject = {
    { &find_object::FindObject::staticMetaObject, qt_meta_stringdata_FindObjectROS,
      qt_meta_data_FindObjectROS, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &FindObjectROS::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *FindObjectROS::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *FindObjectROS::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_FindObjectROS))
        return static_cast<void*>(const_cast< FindObjectROS*>(this));
    typedef find_object::FindObject QMocSuperClass;
    return QMocSuperClass::qt_metacast(_clname);
}

int FindObjectROS::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    typedef find_object::FindObject QMocSuperClass;
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
QT_END_MOC_NAMESPACE
