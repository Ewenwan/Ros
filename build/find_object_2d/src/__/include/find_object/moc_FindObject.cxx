/****************************************************************************
** Meta object code from reading C++ file 'FindObject.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/find_object_2d/include/find_object/FindObject.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'FindObject.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_find_object__FindObject[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      25,   24,   24,   24, 0x05,

 // slots: signature, parameters, type, tag, flags
      84,   66,   24,   24, 0x0a,
     133,  124,   24,   24, 0x2a,
     171,  165,   24,   24, 0x2a,
     202,  199,   24,   24, 0x0a,
     229,  165,   24,   24, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_find_object__FindObject[] = {
    "find_object::FindObject\0\0"
    "objectsFound(find_object::DetectionInfo)\0"
    "image,id,filePath\0"
    "addObjectAndUpdate(cv::Mat,int,QString)\0"
    "image,id\0addObjectAndUpdate(cv::Mat,int)\0"
    "image\0addObjectAndUpdate(cv::Mat)\0id\0"
    "removeObjectAndUpdate(int)\0detect(cv::Mat)\0"
};

void find_object::FindObject::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        FindObject *_t = static_cast<FindObject *>(_o);
        switch (_id) {
        case 0: _t->objectsFound((*reinterpret_cast< const find_object::DetectionInfo(*)>(_a[1]))); break;
        case 1: _t->addObjectAndUpdate((*reinterpret_cast< const cv::Mat(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3]))); break;
        case 2: _t->addObjectAndUpdate((*reinterpret_cast< const cv::Mat(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->addObjectAndUpdate((*reinterpret_cast< const cv::Mat(*)>(_a[1]))); break;
        case 4: _t->removeObjectAndUpdate((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->detect((*reinterpret_cast< const cv::Mat(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData find_object::FindObject::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject find_object::FindObject::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_find_object__FindObject,
      qt_meta_data_find_object__FindObject, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &find_object::FindObject::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *find_object::FindObject::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *find_object::FindObject::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_find_object__FindObject))
        return static_cast<void*>(const_cast< FindObject*>(this));
    return QObject::qt_metacast(_clname);
}

int find_object::FindObject::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void find_object::FindObject::objectsFound(const find_object::DetectionInfo & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
