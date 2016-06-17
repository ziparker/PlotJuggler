#include "dataload_csv.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include "selectxaxisdialog.h"
#include <QDebug>

DataLoadCSV::DataLoadCSV()
{
    _extensions.push_back( "csv");
}

const std::vector<const char*> &DataLoadCSV::compatibleFileExtensions() const
{
    return _extensions;
}

int DataLoadCSV::parseHeader(QFile *file,
                             std::vector<std::pair<bool,QString> >& ordered_names,
                             std::function<void(int)> updateCompletion)
{
    QTextStream inA(file);

    QString first_line = inA.readLine();
    QString second_line = inA.readLine();

    int linecount = 1;

    QStringList string_items = first_line.split(',');
    QStringList secondline_items = second_line.split(',');

    for (int i=0; i < string_items.size(); i++ )
    {
        // remove annoying prefix
        QStringRef field_name ( &string_items[i] );
        if( field_name.startsWith( "field." ) )
        {
            field_name = field_name.mid(6);
        }

        QString qname = field_name.toString();
        ordered_names.push_back( std::make_pair(true,qname) );
    }

    for (unsigned i=0; i < ordered_names.size(); i++ )
    {
        QString field_name ( ordered_names[i].second );
        if( field_name.endsWith( ".name" ) && i < ordered_names.size()-1)
        {
            QString prefix =  field_name.left(field_name.size() - 5 );
            QString replace_name = secondline_items[i];

            ordered_names[i].first = false;
            i++;
            //----------------------------------------

            QString value_prefix =  prefix + ".value";

            if( value_prefix.contains("vectors3d") )
            {
                ordered_names[i].second =  prefix + "." + replace_name + ".x";
                i++;
                ordered_names[i].second =  prefix + "." + replace_name + ".y";
                i++;
                ordered_names[i].second =  prefix + "." + replace_name + ".z";
            }
            else
            {
                while( i < ordered_names.size() &&
                       ordered_names[i].second.startsWith( value_prefix ) )
                {
                    QString name = ordered_names[i].second;
                    QString suffix = name.right( name.size() - value_prefix.size());
                    ordered_names[i].second = prefix + "." + replace_name;
                    if( suffix.size() > 0)
                    {
                       ordered_names[i].second.append( "." + suffix );
                    }
                    i++;
                }
                i--;
            }
        }
    }

    while (!inA.atEnd())
    {
        inA.readLine();
        linecount++;
        if(linecount%100 == 0)
        {
            updateCompletion(0);
        }
    }

    return linecount;
}

PlotDataMap DataLoadCSV::readDataFromFile(QFile *file,
                                          std::function<void(int)> updateCompletion,
                                          std::function<bool()> checkInterruption,
                                          int time_index )
{
    PlotDataMap plot_data;


    std::vector<std::pair<bool, QString> > ordered_names;

    int linecount = parseHeader(file, ordered_names, updateCompletion);

    file->close();
    file->open(QFile::ReadOnly);
    QTextStream inB( file );

    std::vector<SharedVector> ordered_vectors;


    bool interrupted = false;

    int tot_lines = linecount -1;
    linecount = 0;

    double prev_time = -1;

    SharedVector time_vector (new std::vector<double>() );
    time_vector->reserve( tot_lines );

    bool first_line = true;

    while (!inB.atEnd())
    {
        QString line = inB.readLine();

        QStringList string_items = line.split(',');
        QStringList valid_field_names;

        if( first_line )
        {
            for (unsigned i=0; i < ordered_names.size(); i++ )
            {
                bool valid = ordered_names[i].first;
                if( valid )
                {
                    QString& qname = ( ordered_names[i].second );
                    std::string name = qname.toStdString();

                    SharedVector data_vector( new std::vector<double>());
                    data_vector->reserve(tot_lines);

                    ordered_vectors.push_back( data_vector );

                    PlotDataPtr plot( new PlotData );
                    plot->setName( name );
                    plot_data.insert( std::make_pair( name, plot ) );

                    valid_field_names.push_back( qname );
                }
            }

            if( time_index == TIME_INDEX_NOT_DEFINED)
            {
                QStringList field_names;
                field_names.push_back( "INDEX (auto-generated)" );
                field_names.append( valid_field_names );

                SelectXAxisDialog* dialog = new SelectXAxisDialog( &field_names );
                dialog->exec();
                time_index = dialog->getSelectedRowNumber().at(0) -1; // vector is supposed to have only one element
            }

            first_line = false;
        }
        else{
            if( time_index < 0)
            {
                time_vector->push_back( linecount );
            }
            else{
                double t = string_items[ time_index].toDouble();
                if( t <= prev_time)
                {
                    QMessageBox::StandardButton reply;
                    reply = QMessageBox::question(0, tr("Error reading file"),
                                                  tr("Selected time in notstrictly  monotonic. Do you want to abort?\n"
                                                     "(Clicking \"NO\" you continue loading)") );

                    interrupted = (reply == QMessageBox::Yes);
                    break;
                }
                prev_time = t;
                time_vector->push_back( t );
            }

            int index = 0;
            for (int i=0; i < string_items.size(); i++ )
            {
                if( ordered_names[i].first )
                {
                    double y = string_items[i].toDouble();
                    ordered_vectors[index]->push_back( y );
                    index++;
                }

            }

            if(linecount++ %100 == 0)
            {
                updateCompletion( (100* linecount) / tot_lines );
                interrupted = checkInterruption();
                if( interrupted )
                {
                    break;
                }
            }
        }
    }
    file->close();

    if(interrupted)
    {
        plot_data.erase( plot_data.begin(), plot_data.end() );
    }
    else{

        int index = 0;
        for( unsigned i=0; i < ordered_names.size(); i++)
        {
            bool valid = ordered_names[i].first;
            QString name = ordered_names[i].second;
            if( valid )
            {
                plot_data[ name.toStdString() ]->addData( time_vector, ordered_vectors[index]);
                index++;
            }
        }
    }

    return plot_data;
}



DataLoadCSV::~DataLoadCSV()
{

}
