#include "dataload_csv.h"
#include <QTextStream>
#include <QFile>
#include <QMessageBox>
#include "selectxaxisdialog.h"

DataLoadCSV::DataLoadCSV()
{
    _extensions.push_back( "csv");
}

const std::vector<const char*> &DataLoadCSV::compatibleFileExtensions() const
{
    return _extensions;
}

PlotDataMap DataLoadCSV::readDataFromFile(QFile *file,
                                       std::function<void(int)> updateCompletion,
                                       std::function<bool()> checkInterruption)
{
    PlotDataMap plot_data;

    int linecount = 0;
    std::vector<QString> ordered_names;

    bool first_line = true;

    {
        QTextStream inA(file);

        while (!inA.atEnd())
        {
            inA.readLine();
            linecount++;
            if(linecount%100 == 0)
            {
                updateCompletion(0);
            }
        }
        file->close();
    }

    file->open(QFile::ReadOnly);
    QTextStream inB( file );

    std::vector<SharedVector> ordered_vectors;


    bool interrupted = false;

    int tot_lines = linecount -1;
    linecount = 0;
    int time_index = -1;

    double prev_time = -1;

    SharedVector time_vector (new std::vector<double>() );
    time_vector->reserve( tot_lines );

    while (!inB.atEnd())
    {
        QString line = inB.readLine();

        QStringList string_items = line.split(',');

        if( first_line )
        {
            for (int i=0; i < string_items.size(); i++ )
            {
                QStringRef field_name ( &string_items[i] );
                if( field_name.startsWith( "field." ) )
                {
                    field_name = field_name.mid(6);
                }

                QString qname = field_name.toString();
                std::string name = qname.toStdString();

                SharedVector data_vector( new std::vector<double>());
                data_vector->reserve(tot_lines);

                ordered_vectors.push_back( data_vector );
                ordered_names.push_back( qname );

                PlotDataPtr plot( new PlotData );
                plot->setName( name );

                plot_data.insert( std::make_pair( name, plot ) );

            }
            SelectXAxisDialog* dialog = new SelectXAxisDialog( &string_items );
            dialog->exec();
            time_index = dialog->getSelectedRowNumber();

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

            for (int i=0; i < string_items.size(); i++ )
            {
                double y = string_items[i].toDouble();
                ordered_vectors[i]->push_back( y );
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
        for( unsigned i=0; i < ordered_vectors.size(); i++)
        {
            QString name = ordered_names[i];
            plot_data[ name.toStdString() ]->addData( time_vector, ordered_vectors[i]);
        }
    }

    return plot_data;
}



DataLoadCSV::~DataLoadCSV()
{

}
