#include "rule_editing.h"
#include "ui_rule_editing.h"
#include <QDomDocument>
#include <QSettings>
#include <QPlainTextEdit>
#include <QTimer>

XMLSyntaxHighlighter::XMLSyntaxHighlighter(QObject * parent) :
    QSyntaxHighlighter(parent)
{
    setRegexes();
    setFormats();
}

XMLSyntaxHighlighter::XMLSyntaxHighlighter(QTextDocument * parent) :
    QSyntaxHighlighter(parent)
{
    setRegexes();
    setFormats();
}

XMLSyntaxHighlighter::XMLSyntaxHighlighter(QTextEdit * parent) :
    QSyntaxHighlighter(parent)
{
    setRegexes();
    setFormats();
}

void XMLSyntaxHighlighter::highlightBlock(const QString & text)
{
    // Special treatment for xml element regex as we use captured text to emulate lookbehind
    int xmlElementIndex = _xmlElementRegex.indexIn(text);
    while(xmlElementIndex >= 0)
    {
        int matchedPos = _xmlElementRegex.pos(1);
        int matchedLength = _xmlElementRegex.cap(1).length();
        setFormat(matchedPos, matchedLength, _xmlElementFormat);

        xmlElementIndex = _xmlElementRegex.indexIn(text, matchedPos + matchedLength);
    }

    // Highlight xml keywords *after* xml elements to fix any occasional / captured into the enclosing element
    typedef QList<QRegExp>::const_iterator Iter;
    Iter xmlKeywordRegexesEnd = _xmlKeywordRegexes.end();
    for(Iter it = _xmlKeywordRegexes.begin(); it != xmlKeywordRegexesEnd; ++it) {
        const QRegExp & regex = *it;
        highlightByRegex(_xmlKeywordFormat, regex, text);
    }

    highlightByRegex(_xmlAttributeFormat, _xmlAttributeRegex, text);
    highlightByRegex(_xmlCommentFormat, _xmlCommentRegex, text);
    highlightByRegex(_xmlValueFormat, _xmlValueRegex, text);
}

void XMLSyntaxHighlighter::highlightByRegex(const QTextCharFormat & format,
                                                 const QRegExp & regex,
                                            const QString & text)
{
    int index = regex.indexIn(text);
    while(index >= 0)
    {
        int matchedLength = regex.matchedLength();
        setFormat(index, matchedLength, format);
        index = regex.indexIn(text, index + matchedLength);
    }
}

void XMLSyntaxHighlighter::setRegexes()
{
    _xmlElementRegex.setPattern("<[\\s]*[/]?[\\s]*([^\\n]\\w*)(?=[\\s/>])");
    _xmlAttributeRegex.setPattern("\\w+(?=\\=)");
    _xmlValueRegex.setPattern("\"[^\\n\"]+\"(?=[\\s/>])");
    _xmlCommentRegex.setPattern("<!--[^\\n]*-->");

    _xmlKeywordRegexes = QList<QRegExp>() << QRegExp("<\\?") << QRegExp("/>")
                                           << QRegExp(">") << QRegExp("<") << QRegExp("</")
                                           << QRegExp("\\?>");
}

void XMLSyntaxHighlighter::setFormats()
{
    _xmlKeywordFormat.setForeground(Qt::blue);
    _xmlKeywordFormat.setFontWeight(QFont::Normal);

    _xmlElementFormat.setForeground(Qt::darkMagenta);
    _xmlElementFormat.setFontWeight(QFont::Normal);

    _xmlAttributeFormat.setForeground(Qt::darkGreen);
    _xmlAttributeFormat.setFontWeight(QFont::Normal);
    _xmlAttributeFormat.setFontItalic(true);

    _xmlValueFormat.setForeground(Qt::darkRed);

    _xmlCommentFormat.setForeground(Qt::gray);
}

RuleEditing::RuleEditing(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::RuleEditing)
{
    ui->setupUi(this);

    _highlighter = new XMLSyntaxHighlighter(ui->textEdit);

    on_pushButtonReset_pressed();

    _timer.setInterval(200);
    _timer.setSingleShot(false);
    _timer.start();

    connect(&_timer, SIGNAL(timeout()), this, SLOT(on_timer()));
}

RuleEditing::~RuleEditing()
{
    delete ui;
}

bool RuleEditing::isValidXml()
{
    QString errorStr;
    int errorLine, errorColumn;
    bool valid = true;

    QDomDocument domDocument;
    QString text = ui->textEdit->toPlainText();

    if (!domDocument.setContent(text , true,
                                &errorStr, &errorLine, &errorColumn))
    {
        ui->labelValidSyntax->setText("Invalid XML: " + errorStr);
        return false;
    }
    QDomElement root = domDocument.namedItem("SubstitutionRules").toElement();

    if( root.isNull() ){
        errorStr = tr("the root node should be <SubstitutionRules>");
        ui->labelValidSyntax->setText(tr("Invalid: ") + errorStr);
        return false;
    }

    for ( auto type_el = root.firstChildElement( )  ;
          type_el.isNull() == false;
          type_el = type_el.nextSiblingElement() )
    {
        if( type_el.nodeName() != "RosType")
        {
            errorStr = tr("<SubstitutionRules> must have children named <RosType>");
            ui->labelValidSyntax->setText(tr("Invalid: ") + errorStr);
            return false;
        }
        if( type_el.hasAttribute("name") == false)
        {
            errorStr = tr("node <RosType> must have the attribute [name]");
            ui->labelValidSyntax->setText(tr("Invalid: ") + errorStr);
            return false;
        }

        for ( auto rule_el = type_el.firstChildElement( )  ;
              rule_el.isNull() == false;
              rule_el = rule_el.nextSiblingElement( ) )
        {
            if( rule_el.nodeName() != "rule")
            {
                errorStr = tr("<RosType> must have children named <rule>");
                ui->labelValidSyntax->setText(tr("Invalid: ") + errorStr);
                return false;
            }

            auto pattern_el      = rule_el.firstChildElement("pattern");
            auto alias_el        = rule_el.firstChildElement("alias");
            auto substitution_el = rule_el.firstChildElement("substitution");

            if(pattern_el.isNull() ){
                errorStr = tr("<rule> needs a child called <pattern>");
                ui->labelValidSyntax->setText(tr("Invalid: ") + errorStr);
                return false;
            }
            else if( alias_el.isNull() ){
                errorStr = tr("<rule> needs a child called <alias>");
                ui->labelValidSyntax->setText(tr("Invalid: ") + errorStr);
                return false;
            }
            else if( substitution_el.isNull() ){
                errorStr = tr("<rule> needs a child called <substitution>");
                ui->labelValidSyntax->setText(tr("Invalid: ") + errorStr);
                return false;
            }
        }
    }
    ui->labelValidSyntax->setText(tr("Valid") + errorStr);
    return true;
}

void RuleEditing::on_pushButtonSave_pressed()
{
    QSettings settings( "IcarusTechnology", "PlotJuggler");
    settings.setValue("RuleEditing.text", ui->textEdit->toPlainText() );
}

void RuleEditing::on_pushButtonReset_pressed()
{
    QSettings settings( "IcarusTechnology", "PlotJuggler");
    if( settings.contains("RuleEditing.text") )
    {
        QString text = settings.value("RuleEditing.text").toString();
        ui->textEdit->setPlainText(text);
    }
}

void RuleEditing::on_timer()
{
    bool valid = isValidXml();
    ui->pushButtonSave->setEnabled(valid);
}
