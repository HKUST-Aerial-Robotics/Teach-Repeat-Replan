#include "dji_vehicle.hpp"
#include <QComboBox>
#include <QItemDelegate>
#include <QScrollBar>
#include <QTextBrowser>

//! @note widget for GUI
class TurnModeDelegate : public QItemDelegate
{
  Q_OBJECT
public:
  TurnModeDelegate(QObject* parent = 0)
    : QItemDelegate(parent)
  {
  }
  QWidget* createEditor(QWidget*                    parent,
                        const QStyleOptionViewItem& option __UNUSED,
                        const QModelIndex& index __UNUSED) const
  {
    QComboBox* editor = new QComboBox(parent);
    editor->addItem("Clockwise");
    editor->addItem("Counter-clockwise");
    return editor;
  }
  void setEditorData(QWidget* editor, const QModelIndex& index) const
  {
    QString    text     = index.model()->data(index, Qt::EditRole).toString();
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    int        tindex   = comboBox->findText(text);
    comboBox->setCurrentIndex(tindex);
  }
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const
  {
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    QString    text     = comboBox->currentText();
    model->setData(index, text, Qt::EditRole);
  }
  void updateEditorGeometry(QWidget* editor, const QStyleOptionViewItem& option,
                            const QModelIndex& index __UNUSED) const
  {
    editor->setGeometry(option.rect);
  }
};

class ActionDelegate : public QItemDelegate
{
  Q_OBJECT
public:
  ActionDelegate(QObject* parent = 0)
    : QItemDelegate(parent)
  {
  }
  QWidget* createEditor(QWidget*                    parent,
                        const QStyleOptionViewItem& option __UNUSED,
                        const QModelIndex& index __UNUSED) const
  {
    QComboBox* editor = new QComboBox(parent);
    editor->addItem("Stay");
    editor->addItem("Take picture");
    editor->addItem("Start recording");
    editor->addItem("Stop recording");
    editor->addItem("Yaw");
    editor->addItem("Gimbal pitch");
    return editor;
  }
  void setEditorData(QWidget* editor, const QModelIndex& index) const
  {
    QString    text     = index.model()->data(index, Qt::EditRole).toString();
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    int        tindex   = comboBox->findText(text);
    comboBox->setCurrentIndex(tindex);
  }
  void setModelData(QWidget* editor, QAbstractItemModel* model,
                    const QModelIndex& index) const
  {
    QComboBox* comboBox = static_cast<QComboBox*>(editor);
    QString    text     = comboBox->currentText();
    model->setData(index, text, Qt::EditRole);
  }
  void updateEditorGeometry(QWidget*                    editor,
                            const QStyleOptionViewItem& option __UNUSED,
                            const QModelIndex& index __UNUSED) const
  {
    editor->setGeometry(option.rect);
  }
};

class ReadOnlyDelegate : public QItemDelegate
{
  Q_OBJECT
public:
  ReadOnlyDelegate(QObject* parent = 0)
    : QItemDelegate(parent)
  {
  }
  QWidget* createEditor(QWidget* parent             __UNUSED,
                        const QStyleOptionViewItem& option __UNUSED,
                        const QModelIndex& index __UNUSED) const
  {
    return NULL;
  }
};
