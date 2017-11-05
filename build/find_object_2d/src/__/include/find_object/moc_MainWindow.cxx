/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../../src/find_object_2d/include/find_object/MainWindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_find_object__MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      34,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      25,   24,   24,   24, 0x05,

 // slots: signature, parameters, type, tag, flags
      66,   24,   24,   24, 0x0a,
      84,   24,   24,   24, 0x0a,
     101,   24,   24,   24, 0x0a,
     125,  119,   24,   24, 0x0a,
     141,   24,   24,   24, 0x08,
     155,   24,   24,   24, 0x08,
     169,   24,   24,   24, 0x08,
     184,   24,   24,   24, 0x08,
     199,   24,   24,   24, 0x08,
     218,   24,  213,   24, 0x08,
     232,   24,   24,   24, 0x08,
     249,   24,   24,   24, 0x08,
     266,   24,   24,   24, 0x08,
     297,  287,   24,   24, 0x08,
     330,   24,   24,   24, 0x08,
     370,  352,   24,   24, 0x08,
     408,  287,   24,   24, 0x08,
     439,   24,   24,   24, 0x08,
     459,   24,   24,   24, 0x08,
     486,   24,   24,   24, 0x08,
     519,   24,   24,   24, 0x08,
     549,  542,   24,   24, 0x08,
     590,  587,   24,   24, 0x08,
     608,   24,   24,   24, 0x08,
     627,   24,   24,   24, 0x08,
     647,   24,   24,   24, 0x08,
     666,   24,   24,   24, 0x08,
     685,   24,   24,   24, 0x08,
     707,   24,   24,   24, 0x08,
     729,   24,   24,   24, 0x08,
     751,  745,   24,   24, 0x08,
     794,  788,   24,   24, 0x08,
     821,  815,   24,   24, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_find_object__MainWindow[] = {
    "find_object::MainWindow\0\0"
    "objectsFound(find_object::DetectionInfo)\0"
    "startProcessing()\0stopProcessing()\0"
    "pauseProcessing()\0image\0update(cv::Mat)\0"
    "loadSession()\0saveSession()\0loadSettings()\0"
    "saveSettings()\0loadObjects()\0bool\0"
    "saveObjects()\0loadVocabulary()\0"
    "saveVocabulary()\0addObjectFromScene()\0"
    "fileNames\0addObjectsFromFiles(QStringList)\0"
    "addObjectsFromFiles()\0image,id,filePath\0"
    "addObjectFromTcp(cv::Mat,int,QString)\0"
    "loadSceneFromFile(QStringList)\0"
    "loadSceneFromFile()\0setupCameraFromVideoFile()\0"
    "setupCameraFromImagesDirectory()\0"
    "setupCameraFromTcpIp()\0object\0"
    "removeObject(find_object::ObjWidget*)\0"
    "id\0removeObject(int)\0removeAllObjects()\0"
    "updateObjectsSize()\0updateMirrorView()\0"
    "showHideControls()\0showObjectsFeatures()\0"
    "hideObjectsFeatures()\0updateObjects()\0"
    "param\0notifyParametersChanged(QStringList)\0"
    "frame\0moveCameraFrame(int)\0objId\0"
    "rectHovered(int)\0"
};

void find_object::MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->objectsFound((*reinterpret_cast< const find_object::DetectionInfo(*)>(_a[1]))); break;
        case 1: _t->startProcessing(); break;
        case 2: _t->stopProcessing(); break;
        case 3: _t->pauseProcessing(); break;
        case 4: _t->update((*reinterpret_cast< const cv::Mat(*)>(_a[1]))); break;
        case 5: _t->loadSession(); break;
        case 6: _t->saveSession(); break;
        case 7: _t->loadSettings(); break;
        case 8: _t->saveSettings(); break;
        case 9: _t->loadObjects(); break;
        case 10: { bool _r = _t->saveObjects();
            if (_a[0]) *reinterpret_cast< bool*>(_a[0]) = _r; }  break;
        case 11: _t->loadVocabulary(); break;
        case 12: _t->saveVocabulary(); break;
        case 13: _t->addObjectFromScene(); break;
        case 14: _t->addObjectsFromFiles((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 15: _t->addObjectsFromFiles(); break;
        case 16: _t->addObjectFromTcp((*reinterpret_cast< const cv::Mat(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2])),(*reinterpret_cast< const QString(*)>(_a[3]))); break;
        case 17: _t->loadSceneFromFile((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 18: _t->loadSceneFromFile(); break;
        case 19: _t->setupCameraFromVideoFile(); break;
        case 20: _t->setupCameraFromImagesDirectory(); break;
        case 21: _t->setupCameraFromTcpIp(); break;
        case 22: _t->removeObject((*reinterpret_cast< find_object::ObjWidget*(*)>(_a[1]))); break;
        case 23: _t->removeObject((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 24: _t->removeAllObjects(); break;
        case 25: _t->updateObjectsSize(); break;
        case 26: _t->updateMirrorView(); break;
        case 27: _t->showHideControls(); break;
        case 28: _t->showObjectsFeatures(); break;
        case 29: _t->hideObjectsFeatures(); break;
        case 30: _t->updateObjects(); break;
        case 31: _t->notifyParametersChanged((*reinterpret_cast< const QStringList(*)>(_a[1]))); break;
        case 32: _t->moveCameraFrame((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 33: _t->rectHovered((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData find_object::MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject find_object::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_find_object__MainWindow,
      qt_meta_data_find_object__MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &find_object::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *find_object::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *find_object::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_find_object__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int find_object::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 34)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 34;
    }
    return _id;
}

// SIGNAL 0
void find_object::MainWindow::objectsFound(const find_object::DetectionInfo & _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
