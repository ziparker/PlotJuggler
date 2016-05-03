#include "dataload_fb.h"
#include <QDataStream>
#include <QByteArray>
#include <QFile>
#include <QMessageBox>
#include "selectxaxisdialog.h"
#include <QDebug>
#include <iostream>
#include <sstream>
#include "flatbuffers/reflection.h"
#include "flatbuffers/idl.h"

DataLoadFlatbuffer::DataLoadFlatbuffer()
{
    _extensions.push_back( "fb");
    _extensions.push_back( "dtl");
}

const std::vector<const char*> &DataLoadFlatbuffer::compatibleFileExtensions() const
{
    return _extensions;
}


QString extractSchemaFromHeader(QDataStream* stream )
{
    QString header, line;
    do{
        line.clear();
        qint8 c;
        do{
            *stream >> c;
            line.append( c );
        }while ( c !='\n');

        header.append( line );
    }
    while (line.contains( "root_type" ) == false );

    qDebug() << header;
    return header;
}

std::vector<uint8_t> parseSchema(QString schema_text)
{
    flatbuffers::Parser parser;
    bool ok = parser.Parse( schema_text.toStdString().c_str(), nullptr, nullptr );

    if( !ok ) {
        qDebug() << parser.error_.c_str();
    }

    assert("Parse failed " && ok);

    parser.Serialize();

    auto data_ptr = parser.builder_.GetBufferPointer();
    auto length = parser.builder_.GetSize();

    return std::vector<uint8_t>(  &data_ptr[0], &data_ptr[length] );
}


bool getFlatbuffer(QDataStream* source, std::vector<uint8_t>* destination)
{
    if( source->atEnd())
    {
        return false;
    }
    quint8 c;
    uint32_t chunk_size = 0;
    for (int i=0; i< sizeof(uint32_t); i++ )
    {
        (*source) >> c;
        chunk_size += c << (8*i);
    }

    std::cout << std::endl;
    destination->resize( chunk_size );

    if( destination)    {
        for (uint32_t i=0; i< chunk_size; i++ ) {
            (*source) >> (*destination)[i];
        }
    }
    else{
        source->skipRawData( chunk_size );
    }
    return true;
}

std::vector<std::string> getFieldNamesFromSchema(const reflection::Schema* schema,
                                                 flatbuffers::Table *root_table )
{
    std::vector<std::string> field_names;
    auto fields = schema->root_table()->fields();

    for (uint32_t f=0; f< fields->size(); f++)
    {
        auto field = fields->Get(f);
        const char* field_name = field->name()->c_str();

        if( field->type()->base_type() == reflection::Vector)
        {
            const auto& vect = GetFieldAnyV( *root_table, *field );

            for (int v=0; v < vect->size(); v++)
            {
                std::stringstream ss;
                ss << field_name << "." << v;
                field_names.push_back( ss.str() );
            }
        }
        else{
            field_names.push_back( std::string(field_name)  );
        }
    }
    return field_names;
}


PlotDataMap DataLoadFlatbuffer::readDataFromFile(QFile *file,
                                                 std::function<void(int)> updateCompletion,
                                                 std::function<bool()> checkInterruption)
{
    PlotDataMap plot_data;

    // get the schema from the header of the file
    QDataStream file_stream(file);

    QString schema_text = extractSchemaFromHeader( &file_stream );

    // parse the schema
    std::vector<uint8_t> schema_buffer = parseSchema( schema_text );
    const reflection::Schema* schema = reflection::GetSchema( schema_buffer.data() );

    //--------------------------------------
    // build data

    std::vector<uint8_t> flatbuffer;
    std::vector<std::string> field_names;

    auto fields = schema->root_table()->fields();

    std::vector<SharedVector> data_ptr;

    bool first_pass = true;

    while( getFlatbuffer( &file_stream, &flatbuffer) )
    {
        auto root_table = flatbuffers::GetAnyRoot( flatbuffer.data() );

        // at the first flatbuffer, create a vector with the names.
        if( first_pass)
        {
            field_names = getFieldNamesFromSchema( schema, root_table );

            for (int i=0; i< field_names.size(); i++)
            {
                data_ptr.push_back( SharedVector(new std::vector<double>()) );
            }
            first_pass = false;
        }

        int index = 0;

        for (uint32_t f=0; f< fields->size(); f++)
        {
            auto field = fields->Get(f);

            SharedVector data_vector;

            if( field->type()->base_type() == reflection::Vector)
            {
                const auto& vect = GetFieldAnyV( *root_table, *field );
                const auto& vect_type = field->type()->element();

                for (int v=0; v < vect->size(); v++)
                {
                    data_vector = data_ptr[index];
                    index++;

                    double value = GetAnyVectorElemF( vect, vect_type, v);
                    data_vector->push_back( value );
                }
            }
            else{
                data_vector = data_ptr[index];
                index++;

                double value = GetAnyFieldF(*root_table, *field );
                data_vector->push_back( value );
            }
        }
    }

    return plot_data;
}



DataLoadFlatbuffer::~DataLoadFlatbuffer()
{

}
