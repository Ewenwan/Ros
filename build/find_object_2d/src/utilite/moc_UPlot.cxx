/****************************************************************************
** Meta object code from reading C++ file 'UPlot.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../src/find_object_2d/src/utilite/UPlot.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'UPlot.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_UPlotCurve[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      43,   11,   11,   11, 0x0a,
      59,   51,   11,   11, 0x0a,
      86,   76,   11,   11, 0x0a,
     111,  107,   11,   11, 0x0a,
     133,  128,   11,   11, 0x0a,
     156,  154,   11,   11, 0x0a,
     176,  172,   11,   11, 0x0a,
     198,  154,   11,   11, 0x0a,
     216,  128,   11,   11, 0x0a,
     254,  248,   11,   11, 0x0a,
     298,  295,   11,   11, 0x0a,
     324,  295,   11,   11, 0x0a,
     348,  295,   11,   11, 0x0a,
     378,  295,   11,   11, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_UPlotCurve[] = {
    "UPlotCurve\0\0dataChanged(const UPlotCurve*)\0"
    "clear()\0visible\0setVisible(bool)\0"
    "increment\0setXIncrement(float)\0val\0"
    "setXStart(float)\0data\0addValue(UPlotItem*)\0"
    "y\0addValue(float)\0x,y\0addValue(float,float)\0"
    "addValue(QString)\0addValues(QVector<UPlotItem*>&)\0"
    "xs,ys\0addValues(QVector<float>,QVector<float>)\0"
    "ys\0addValues(QVector<float>)\0"
    "addValues(QVector<int>)\0"
    "addValues(std::vector<float>)\0"
    "addValues(std::vector<int>)\0"
};

void UPlotCurve::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        UPlotCurve *_t = static_cast<UPlotCurve *>(_o);
        switch (_id) {
        case 0: _t->dataChanged((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 1: _t->clear(); break;
        case 2: _t->setVisible((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->setXIncrement((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->setXStart((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->addValue((*reinterpret_cast< UPlotItem*(*)>(_a[1]))); break;
        case 6: _t->addValue((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: _t->addValue((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< float(*)>(_a[2]))); break;
        case 8: _t->addValue((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 9: _t->addValues((*reinterpret_cast< QVector<UPlotItem*>(*)>(_a[1]))); break;
        case 10: _t->addValues((*reinterpret_cast< const QVector<float>(*)>(_a[1])),(*reinterpret_cast< const QVector<float>(*)>(_a[2]))); break;
        case 11: _t->addValues((*reinterpret_cast< const QVector<float>(*)>(_a[1]))); break;
        case 12: _t->addValues((*reinterpret_cast< const QVector<int>(*)>(_a[1]))); break;
        case 13: _t->addValues((*reinterpret_cast< const std::vector<float>(*)>(_a[1]))); break;
        case 14: _t->addValues((*reinterpret_cast< const std::vector<int>(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData UPlotCurve::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject UPlotCurve::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_UPlotCurve,
      qt_meta_data_UPlotCurve, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotCurve::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotCurve::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotCurve::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotCurve))
        return static_cast<void*>(const_cast< UPlotCurve*>(this));
    return QObject::qt_metacast(_clname);
}

int UPlotCurve::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    }
    return _id;
}

// SIGNAL 0
void UPlotCurve::dataChanged(const UPlotCurve * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
static const uint qt_meta_data_UPlotCurveThreshold[] = {

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
      31,   21,   20,   20, 0x0a,
      63,   51,   20,   20, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_UPlotCurveThreshold[] = {
    "UPlotCurveThreshold\0\0threshold\0"
    "setThreshold(float)\0orientation\0"
    "setOrientation(Qt::Orientation)\0"
};

void UPlotCurveThreshold::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        UPlotCurveThreshold *_t = static_cast<UPlotCurveThreshold *>(_o);
        switch (_id) {
        case 0: _t->setThreshold((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 1: _t->setOrientation((*reinterpret_cast< Qt::Orientation(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData UPlotCurveThreshold::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject UPlotCurveThreshold::staticMetaObject = {
    { &UPlotCurve::staticMetaObject, qt_meta_stringdata_UPlotCurveThreshold,
      qt_meta_data_UPlotCurveThreshold, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotCurveThreshold::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotCurveThreshold::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotCurveThreshold::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotCurveThreshold))
        return static_cast<void*>(const_cast< UPlotCurveThreshold*>(this));
    return UPlotCurve::qt_metacast(_clname);
}

int UPlotCurveThreshold::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = UPlotCurve::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    }
    return _id;
}
static const uint qt_meta_data_UPlotLegendItem[] = {

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
      17,   16,   16,   16, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_UPlotLegendItem[] = {
    "UPlotLegendItem\0\0legendItemRemoved(const UPlotCurve*)\0"
};

void UPlotLegendItem::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        UPlotLegendItem *_t = static_cast<UPlotLegendItem *>(_o);
        switch (_id) {
        case 0: _t->legendItemRemoved((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData UPlotLegendItem::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject UPlotLegendItem::staticMetaObject = {
    { &QPushButton::staticMetaObject, qt_meta_stringdata_UPlotLegendItem,
      qt_meta_data_UPlotLegendItem, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotLegendItem::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotLegendItem::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotLegendItem::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotLegendItem))
        return static_cast<void*>(const_cast< UPlotLegendItem*>(this));
    return QPushButton::qt_metacast(_clname);
}

int UPlotLegendItem::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QPushButton::qt_metacall(_c, _id, _a);
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
void UPlotLegendItem::legendItemRemoved(const UPlotCurve * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
static const uint qt_meta_data_UPlotLegend[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      19,   13,   12,   12, 0x05,
      70,   56,   12,   12, 0x05,

 // slots: signature, parameters, type, tag, flags
     112,   13,   12,   12, 0x0a,
     148,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_UPlotLegend[] = {
    "UPlotLegend\0\0curve\0"
    "legendItemRemoved(const UPlotCurve*)\0"
    "curve,toggled\0legendItemToggled(const UPlotCurve*,bool)\0"
    "removeLegendItem(const UPlotCurve*)\0"
    "redirectToggled(bool)\0"
};

void UPlotLegend::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        UPlotLegend *_t = static_cast<UPlotLegend *>(_o);
        switch (_id) {
        case 0: _t->legendItemRemoved((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 1: _t->legendItemToggled((*reinterpret_cast< const UPlotCurve*(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->removeLegendItem((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 3: _t->redirectToggled((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData UPlotLegend::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject UPlotLegend::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_UPlotLegend,
      qt_meta_data_UPlotLegend, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlotLegend::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlotLegend::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlotLegend::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlotLegend))
        return static_cast<void*>(const_cast< UPlotLegend*>(this));
    return QWidget::qt_metacast(_clname);
}

int UPlotLegend::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void UPlotLegend::legendItemRemoved(const UPlotCurve * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void UPlotLegend::legendItemToggled(const UPlotCurve * _t1, bool _t2)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
static const uint qt_meta_data_UOrientableLabel[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

static const char qt_meta_stringdata_UOrientableLabel[] = {
    "UOrientableLabel\0"
};

void UOrientableLabel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObjectExtraData UOrientableLabel::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject UOrientableLabel::staticMetaObject = {
    { &QLabel::staticMetaObject, qt_meta_stringdata_UOrientableLabel,
      qt_meta_data_UOrientableLabel, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UOrientableLabel::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UOrientableLabel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UOrientableLabel::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UOrientableLabel))
        return static_cast<void*>(const_cast< UOrientableLabel*>(this));
    return QLabel::qt_metacast(_clname);
}

int UOrientableLabel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QLabel::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
static const uint qt_meta_data_UPlot[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,    7,    6,    6, 0x0a,
      56,   44,    6,    6, 0x0a,
      90,    6,    6,    6, 0x0a,
     103,    6,    6,    6, 0x0a,
     115,    6,    6,    6, 0x08,
     131,    7,    6,    6, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_UPlot[] = {
    "UPlot\0\0curve\0removeCurve(const UPlotCurve*)\0"
    "curve,shown\0showCurve(const UPlotCurve*,bool)\0"
    "updateAxis()\0clearData()\0captureScreen()\0"
    "updateAxis(const UPlotCurve*)\0"
};

void UPlot::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        UPlot *_t = static_cast<UPlot *>(_o);
        switch (_id) {
        case 0: _t->removeCurve((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        case 1: _t->showCurve((*reinterpret_cast< const UPlotCurve*(*)>(_a[1])),(*reinterpret_cast< bool(*)>(_a[2]))); break;
        case 2: _t->updateAxis(); break;
        case 3: _t->clearData(); break;
        case 4: _t->captureScreen(); break;
        case 5: _t->updateAxis((*reinterpret_cast< const UPlotCurve*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData UPlot::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject UPlot::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_UPlot,
      qt_meta_data_UPlot, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &UPlot::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *UPlot::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *UPlot::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_UPlot))
        return static_cast<void*>(const_cast< UPlot*>(this));
    return QWidget::qt_metacast(_clname);
}

int UPlot::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
