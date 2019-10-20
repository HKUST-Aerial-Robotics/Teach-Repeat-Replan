/****************************************************************************
** Meta object code from reading C++ file 'goal_tool.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/goal_tool.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'goal_tool.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_rviz__Goal3DTool[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      18,   17,   17,   17, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_rviz__Goal3DTool[] = {
    "rviz::Goal3DTool\0\0updateTopic()\0"
};

void rviz::Goal3DTool::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        Goal3DTool *_t = static_cast<Goal3DTool *>(_o);
        switch (_id) {
        case 0: _t->updateTopic(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObjectExtraData rviz::Goal3DTool::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject rviz::Goal3DTool::staticMetaObject = {
    { &Pose3DTool::staticMetaObject, qt_meta_stringdata_rviz__Goal3DTool,
      qt_meta_data_rviz__Goal3DTool, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &rviz::Goal3DTool::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *rviz::Goal3DTool::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *rviz::Goal3DTool::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_rviz__Goal3DTool))
        return static_cast<void*>(const_cast< Goal3DTool*>(this));
    return Pose3DTool::qt_metacast(_clname);
}

int rviz::Goal3DTool::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Pose3DTool::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 1)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 1;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
