/****************************************************************************
** Meta object code from reading C++ file 'ImageDropWidget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/src/find_object_2d/src/ImageDropWidget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ImageDropWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_find_object__ImageDropWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      30,   29,   29,   29, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_find_object__ImageDropWidget[] = {
    "find_object::ImageDropWidget\0\0"
    "imagesReceived(QStringList)\0"
};

void find_object::ImageDropWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ImageDropWidget *_t = static_cast<ImageDropWidget *>(_o);
        switch (_id) {
        case 0: _t->imagesReceived((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData find_object::ImageDropWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject find_object::ImageDropWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_find_object__ImageDropWidget,
      qt_meta_data_find_object__ImageDropWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &find_object::ImageDropWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *find_object::ImageDropWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *find_object::ImageDropWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_find_object__ImageDropWidget))
        return static_cast<void*>(const_cast< ImageDropWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int find_object::ImageDropWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void find_object::ImageDropWidget::imagesReceived(const QStringList & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
