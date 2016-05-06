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
#include "lz4.h"

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


bool decompressBlock( QDataStream* source, LZ4_streamDecode_t* lz4_stream, std::vector<char>* output )
{
    qDebug() << "decompress block";

    if( source->atEnd())
    {
        return false;
    }

    const long BUFFER_SIZE = 1024*128 ;
    output->resize( BUFFER_SIZE );

    char comp_buffer[LZ4_COMPRESSBOUND( BUFFER_SIZE )];
    int64_t block_size = 0;

    char c[8];
    source->readRawData( c, 8);

    for (int i=0; i<8; i++ )
    {
        block_size += ((uint8_t)c[i]) << (8*i);
        printf(" %02X ", (uint8_t)c[i]);
    }
    std::cout << std::endl;
    qDebug() << "  block size " << block_size;

    if( block_size <= 0 || block_size > BUFFER_SIZE) {
        return false;
    }

    const size_t readCount =  source->readRawData( comp_buffer, (size_t) block_size);
    if(readCount != (size_t) block_size) {
        return false;
    }

    const int64_t decBytes = LZ4_decompress_safe_continue(
                lz4_stream,
                comp_buffer,
                output->data(),
                block_size,
                BUFFER_SIZE );

    qDebug() << "uncompressed size " << decBytes;

    if(decBytes <= 0) {
        return false;
    }
    output->shrink_to_fit();

    return true;
}


char* getSingleFlatbuffer(char* source, std::vector<uint8_t>* destination)
{
    uint8_t c;
    uint32_t chunk_size = 0;
    for (int i=0; i< sizeof(uint32_t); i++ )
    {
        c = (uint8_t)(*source);
        source++;
        chunk_size += c << (8*i);
    }

    qDebug() <<  "chunk_size " << chunk_size ;

    if( destination) {

        destination->resize( chunk_size );
        for (uint32_t i=0; i< chunk_size; i++ )
        {
            (*destination)[i] = (uint8_t)(*source);
            source++;
        }
    }
    else{
        source += chunk_size ;
    }

    return source;
}

QStringList getFieldNamesFromSchema(const reflection::Schema* schema,
                                    flatbuffers::Table *root_table )
{
    QStringList field_names;
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
                field_names.push_back(  ss.str().c_str() );
            }
        }
        else{
            field_names.push_back( field_name  );
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

    file_stream.device()->setTextModeEnabled( false );

    QString schema_text = extractSchemaFromHeader( &file_stream );

    // parse the schema
    std::vector<uint8_t> schema_buffer = parseSchema( schema_text );
    const reflection::Schema* schema = reflection::GetSchema( schema_buffer.data() );

    //--------------------------------------
    // build data

    std::vector<uint8_t> flatbuffer;
    QStringList field_names;

    auto fields = schema->root_table()->fields();

    std::vector<SharedVector> data_vectors;

    bool first_pass = true;

    int time_index = -1;
    int linecount = 0;

    SharedVector time_vector (new std::vector<double>() );

    bool interrupted = false;

    std::vector<char> decompressed_block;

    LZ4_streamDecode_t lz4_stream;

    LZ4_setStreamDecode( &lz4_stream, NULL, 0);

    while( decompressBlock( &file_stream, &lz4_stream, &decompressed_block ) && !interrupted )
    {
        interrupted = checkInterruption();
        uint64_t block_size = decompressed_block.size();
        char* block_end = &decompressed_block.data()[ block_size ];
        char* block_ptr = &decompressed_block.data()[ 0 ];

        while( block_ptr !=  block_end )
        {
            block_ptr = getSingleFlatbuffer( block_ptr, &flatbuffer );

            qDebug() << linecount << " " << flatbuffer.size() ;

            auto root_table = flatbuffers::GetAnyRoot( flatbuffer.data() );

            // qDebug() << linecount << " " << flatbuffer.size() ;

            // at the first flatbuffer, create a vector with the names.
            if( first_pass)
            {
                field_names = getFieldNamesFromSchema( schema, root_table );

                SelectXAxisDialog* dialog = new SelectXAxisDialog( &field_names );
                dialog->exec();
                time_index = dialog->getSelectedRowNumber();

                for (int i=0; i< field_names.size(); i++)
                {
                    data_vectors.push_back( SharedVector(new std::vector<double>()) );

                    PlotDataPtr plot( new PlotData );
                    std::string name = field_names.at(i).toStdString();
                    plot->setName( name );
                    plot_data.insert( std::make_pair( name, plot ) );
                }
                first_pass = false;
            }

            int index = 0;

            if( time_index < 0) {
                time_vector->push_back( linecount++ );
            }

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
                        data_vector = data_vectors[index];


                        double value = GetAnyVectorElemF( vect, vect_type, v);
                        data_vector->push_back( value );

                        if( time_index == index) {
                            time_vector->push_back( value );
                        }
                        index++;
                    }
                }
                else{
                    data_vector = data_vectors[index];

                    double value = GetAnyFieldF(*root_table, *field );
                    data_vector->push_back( value );

                    if( time_index == index) {
                        time_vector->push_back( value );
                    }
                    index++;
                }
            }
        }
    }


    if(interrupted)
    {
        plot_data.erase( plot_data.begin(), plot_data.end() );
    }
    else{

        for( unsigned i=0; i < field_names.size(); i++)
        {
            QString name = field_names[i];
            plot_data[ name.toStdString() ]->addData( time_vector, data_vectors[i]);
        }
    }

    return plot_data;
}



DataLoadFlatbuffer::~DataLoadFlatbuffer()
{

}
