#include "opencv2/core/core_c.h"
#include "opencv2/ml/ml.hpp"

#include <cstdio>
#include <vector>
/*

*/

static void help()
{
    printf("\nThe sample demonstrates how to train Random Trees classifier\n"
    "(or Boosting classifier, or MLP, or Knearest, or Nbayes, or Support Vector Machines - see main()) using the provided dataset.\n"
    "\n"
    "We use the sample database letter-recognition.data\n"
    "from UCI Repository, here is the link:\n"
    "\n"
    "Newman, D.J. & Hettich, S. & Blake, C.L. & Merz, C.J. (1998).\n"
    "UCI Repository of machine learning databases\n"
    "[http://www.ics.uci.edu/~mlearn/MLRepository.html].\n"
    "Irvine, CA: University of California, Department of Information and Computer Science.\n"
    "\n"
    "The dataset consists of 20000 feature vectors along with the\n"
    "responses - capital latin letters A..Z.\n"
    "The first 16000 (10000 for boosting)) samples are used for training\n"
    "and the remaining 4000 (10000 for boosting) - to test the classifier.\n"
    "======================================================\n");
    printf("\nThis is letter recognition sample.\n"
            "The usage: letter_recog [-data <path to letter-recognition.data>] \\\n"
            "  [-save <output XML file for the classifier>] \\\n"
            "  [-load <XML file with the pre-trained classifier>] \\\n"
            "  [-boost|-mlp|-knearest|-nbayes|-svm] # to use boost/mlp/knearest/SVM classifier instead of default Random Trees\n" );
}

// This function reads data and responses from the file <filename>
static int
read_num_class_data( const char* filename, int var_count,
                     CvMat** data, CvMat** responses )
{
    const int M = 1024;
    FILE* f = fopen( filename, "rt" );
    CvMemStorage* storage;
    CvSeq* seq;
    char buf[M+2];
    float* el_ptr;
    CvSeqReader reader;
    int i, j;

    if( !f )
        return 0;

    el_ptr = new float[var_count+1];
    storage = cvCreateMemStorage();
    seq = cvCreateSeq( 0, sizeof(*seq), (var_count+1)*sizeof(float), storage );

    for(;;)
    {
        char* ptr;
        if( !fgets( buf, M, f ) || !strchr( buf, ',' ) )
            break;
        el_ptr[0] = buf[0];
        ptr = buf+2;
        for( i = 1; i <= var_count; i++ )
        {
            int n = 0;
            sscanf( ptr, "%f%n", el_ptr + i, &n );
            ptr += n + 1;
        }
        if( i <= var_count )
            break;
        cvSeqPush( seq, el_ptr );
    }
    fclose(f);

    *data = cvCreateMat( seq->total, var_count, CV_32F );
    *responses = cvCreateMat( seq->total, 1, CV_32F );

    cvStartReadSeq( seq, &reader );

    for( i = 0; i < seq->total; i++ )
    {
        const float* sdata = (float*)reader.ptr + 1;
        float* ddata = data[0]->data.fl + var_count*i;
        float* dr = responses[0]->data.fl + i;

        for( j = 0; j < var_count; j++ )
            ddata[j] = sdata[j];
        *dr = sdata[-1];
        CV_NEXT_SEQ_ELEM( seq->elem_size, reader );
    }

    cvReleaseMemStorage( &storage );
    delete[] el_ptr;
    return 1;
}

static
int build_rtrees_classifier( char* data_filename,
    char* filename_to_save, char* filename_to_load )
{
    CvMat* data = 0;
    CvMat* responses = 0;
    CvMat* var_type = 0;
    CvMat* sample_idx = 0;

    int ok = read_num_class_data( data_filename, 16, &data, &responses );
    int nsamples_all = 0, ntrain_samples = 0;
    int i = 0;
    double train_hr = 0, test_hr = 0;
    CvRTrees forest;
    CvMat* var_importance = 0;

    if( !ok )
    {
        printf( "Could not read the database %s\n", data_filename );
        return -1;
    }

    printf( "The database %s is loaded.\n", data_filename );
    nsamples_all = data->rows;
    ntrain_samples = (int)(nsamples_all*0.8);

    // Create or load Random Trees classifier
    if( filename_to_load )
    {
        // load classifier from the specified file
        forest.load( filename_to_load );
        ntrain_samples = 0;
        if( forest.get_tree_count() == 0 )
        {
            printf( "Could not read the classifier %s\n", filename_to_load );
            return -1;
        }
        printf( "The classifier %s is loaded.\n", filename_to_load );
    }
    else
    {
        // create classifier by using <data> and <responses>
        printf( "Training the classifier ...\n");

        // 1. create type mask
        var_type = cvCreateMat( data->cols + 1, 1, CV_8U );
        cvSet( var_type, cvScalarAll(CV_VAR_ORDERED) );
        cvSetReal1D( var_type, data->cols, CV_VAR_CATEGORICAL );

        // 2. create sample_idx
        sample_idx = cvCreateMat( 1, nsamples_all, CV_8UC1 );
        {
            CvMat mat;
            cvGetCols( sample_idx, &mat, 0, ntrain_samples );
            cvSet( &mat, cvRealScalar(1) );

            cvGetCols( sample_idx, &mat, ntrain_samples, nsamples_all );
            cvSetZero( &mat );
        }

        // 3. train classifier
        forest.train( data, CV_ROW_SAMPLE, responses, 0, sample_idx, var_type, 0,
            CvRTParams(10,10,0,false,15,0,true,4,100,0.01f,CV_TERMCRIT_ITER));
        printf( "\n");
    }

    // compute prediction error on train and test data
    for( i = 0; i < nsamples_all; i++ )
    {
        double r;
        CvMat sample;
        cvGetRow( data, &sample, i );

        r = forest.predict( &sample );
        r = fabs((double)r - responses->data.fl[i]) <= FLT_EPSILON ? 1 : 0;

        if( i < ntrain_samples )
            train_hr += r;
        else
            test_hr += r;
    }

    test_hr /= (double)(nsamples_all-ntrain_samples);
    train_hr /= (double)ntrain_samples;
    printf( "Recognition rate: train = %.1f%%, test = %.1f%%\n",
            train_hr*100., test_hr*100. );

    printf( "Number of trees: %d\n", forest.get_tree_count() );

    // Print variable importance
    var_importance = (CvMat*)forest.get_var_importance();
    if( var_importance )
    {
        double rt_imp_sum = cvSum( var_importance ).val[0];
        printf("var#\timportance (in %%):\n");
        for( i = 0; i < var_importance->cols; i++ )
            printf( "%-2d\t%-4.1f\n", i,
            100.f*var_importance->data.fl[i]/rt_imp_sum);
    }

    //Print some proximitites
    printf( "Proximities between some samples corresponding to the letter 'T':\n" );
    {
        CvMat sample1, sample2;
        const int pairs[][2] = {{0,103}, {0,106}, {106,103}, {-1,-1}};

        for( i = 0; pairs[i][0] >= 0; i++ )
        {
            cvGetRow( data, &sample1, pairs[i][0] );
            cvGetRow( data, &sample2, pairs[i][1] );
            printf( "proximity(%d,%d) = %.1f%%\n", pairs[i][0], pairs[i][1],
                forest.get_proximity( &sample1, &sample2 )*100. );
        }
    }

    // Save Random Trees classifier to file if needed
    if( filename_to_save )
        forest.save( filename_to_save );

    cvReleaseMat( &sample_idx );
    cvReleaseMat( &var_type );
    cvReleaseMat( &data );
    cvReleaseMat( &responses );

    return 0;
}


static
int build_boost_classifier( char* data_filename,
    char* filename_to_save, char* filename_to_load )
{
    const int class_count = 26;
    CvMat* data = 0;
    CvMat* responses = 0;
    CvMat* var_type = 0;
    CvMat* temp_sample = 0;
    CvMat* weak_responses = 0;

    int ok = read_num_class_data( data_filename, 16, &data, &responses );
    int nsamples_all = 0, ntrain_samples = 0;
    int var_count;
    int i, j, k;
    double train_hr = 0, test_hr = 0;
    CvBoost boost;

    if( !ok )
    {
        printf( "Could not read the database %s\n", data_filename );
        return -1;
    }

    printf( "The database %s is loaded.\n", data_filename );
    nsamples_all = data->rows;
    ntrain_samples = (int)(nsamples_all*0.5);
    var_count = data->cols;

    // Create or load Boosted Tree classifier
    if( filename_to_load )
    {
        // load classifier from the specified file
        boost.load( filename_to_load );
        ntrain_samples = 0;
        if( !boost.get_weak_predictors() )
        {
            printf( "Could not read the classifier %s\n", filename_to_load );
            return -1;
        }
        printf( "The classifier %s is loaded.\n", filename_to_load );
    }
    else
    {
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //
        // As currently boosted tree classifier in MLL can only be trained
        // for 2-class problems, we transform the training database by
        // "unrolling" each training sample as many times as the number of
        // classes (26) that we have.
        //
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        CvMat* new_data = cvCreateMat( ntrain_samples*class_count, var_count + 1, CV_32F );
        CvMat* new_responses = cvCreateMat( ntrain_samples*class_count, 1, CV_32S );

        // 1. unroll the database type mask
        printf( "Unrolling the database...\n");
        for( i = 0; i < ntrain_samples; i++ )
        {
            float* data_row = (float*)(data->data.ptr + data->step*i);
            for( j = 0; j < class_count; j++ )
            {
                float* new_data_row = (float*)(new_data->data.ptr +
                                new_data->step*(i*class_count+j));
                for( k = 0; k < var_count; k++ )
                    new_data_row[k] = data_row[k];
                new_data_row[var_count] = (float)j;
                new_responses->data.i[i*class_count + j] = responses->data.fl[i] == j+'A';
            }
        }

        // 2. create type mask
        var_type = cvCreateMat( var_count + 2, 1, CV_8U );
        cvSet( var_type, cvScalarAll(CV_VAR_ORDERED) );
        // the last indicator variable, as well
        // as the new (binary) response are categorical
        cvSetReal1D( var_type, var_count, CV_VAR_CATEGORICAL );
        cvSetReal1D( var_type, var_count+1, CV_VAR_CATEGORICAL );

        // 3. train classifier
        printf( "Training the classifier (may take a few minutes)...\n");
        boost.train( new_data, CV_ROW_SAMPLE, new_responses, 0, 0, var_type, 0,
            CvBoostParams(CvBoost::REAL, 100, 0.95, 5, false, 0 ));
        cvReleaseMat( &new_data );
        cvReleaseMat( &new_responses );
        printf("\n");
    }

    temp_sample = cvCreateMat( 1, var_count + 1, CV_32F );
    weak_responses = cvCreateMat( 1, boost.get_weak_predictors()->total, CV_32F );

    // compute prediction error on train and test data
    for( i = 0; i < nsamples_all; i++ )
    {
        int best_class = 0;
        double max_sum = -DBL_MAX;
        double r;
        CvMat sample;
        cvGetRow( data, &sample, i );
        for( k = 0; k < var_count; k++ )
            temp_sample->data.fl[k] = sample.data.fl[k];

        for( j = 0; j < class_count; j++ )
        {
            temp_sample->data.fl[var_count] = (float)j;
            boost.predict( temp_sample, 0, weak_responses );
            double sum = cvSum( weak_responses ).val[0];
            if( max_sum < sum )
            {
                max_sum = sum;
                best_class = j + 'A';
            }
        }

        r = fabs(best_class - responses->data.fl[i]) < FLT_EPSILON ? 1 : 0;

        if( i < ntrain_samples )
            train_hr += r;
        else
            test_hr += r;
    }

    test_hr /= (double)(nsamples_all-ntrain_samples);
    train_hr /= (double)ntrain_samples;
    printf( "Recognition rate: train = %.1f%%, test = %.1f%%\n",
            train_hr*100., test_hr*100. );

    printf( "Number of trees: %d\n", boost.get_weak_predictors()->total );

    // Save classifier to file if needed
    if( filename_to_save )
        boost.save( filename_to_save );

    cvReleaseMat( &temp_sample );
    cvReleaseMat( &weak_responses );
    cvReleaseMat( &var_type );
    cvReleaseMat( &data );
    cvReleaseMat( &responses );

    return 0;
}


static
int build_mlp_classifier( char* data_filename,
    char* filename_to_save, char* filename_to_load )
{
    const int class_count = 26;
    CvMat* data = 0;
    CvMat train_data;
    CvMat* responses = 0;
    CvMat* mlp_response = 0;

    int ok = read_num_class_data( data_filename, 16, &data, &responses );
    int nsamples_all = 0, ntrain_samples = 0;
    int i, j;
    double train_hr = 0, test_hr = 0;
    CvANN_MLP mlp;

    if( !ok )
    {
        printf( "Could not read the database %s\n", data_filename );
        return -1;
    }

    printf( "The database %s is loaded.\n", data_filename );
    nsamples_all = data->rows;
    ntrain_samples = (int)(nsamples_all*0.8);

    // Create or load MLP classifier
    if( filename_to_load )
    {
        // load classifier from the specified file
        mlp.load( filename_to_load );
        ntrain_samples = 0;
        if( !mlp.get_layer_count() )
        {
            printf( "Could not read the classifier %s\n", filename_to_load );
            return -1;
        }
        printf( "The classifier %s is loaded.\n", filename_to_load );
    }
    else
    {
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        //
        // MLP does not support categorical variables by explicitly.
        // So, instead of the output class label, we will use
        // a binary vector of <class_count> components for training and,
        // therefore, MLP will give us a vector of "probabilities" at the
        // prediction stage
        //
        // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        CvMat* new_responses = cvCreateMat( ntrain_samples, class_count, CV_32F );

        // 1. unroll the responses
        printf( "Unrolling the responses...\n");
        for( i = 0; i < ntrain_samples; i++ )
        {
            int cls_label = cvRound(responses->data.fl[i]) - 'A';
            float* bit_vec = (float*)(new_responses->data.ptr + i*new_responses->step);
            for( j = 0; j < class_count; j++ )
                bit_vec[j] = 0.f;
            bit_vec[cls_label] = 1.f;
        }
        cvGetRows( data, &train_data, 0, ntrain_samples );

        // 2. train classifier
        int layer_sz[] = { data->cols, 100, 100, class_count };
        CvMat layer_sizes =
            cvMat( 1, (int)(sizeof(layer_sz)/sizeof(layer_sz[0])), CV_32S, layer_sz );
        mlp.create( &layer_sizes );
        printf( "Training the classifier (may take a few minutes)...\n");

#if 1
        int method = CvANN_MLP_TrainParams::BACKPROP;
        double method_param = 0.001;
        int max_iter = 300;
#else
        int method = CvANN_MLP_TrainParams::RPROP;
        double method_param = 0.1;
        int max_iter = 1000;
#endif

        mlp.train( &train_data, new_responses, 0, 0,
            CvANN_MLP_TrainParams(cvTermCriteria(CV_TERMCRIT_ITER,max_iter,0.01),
                                  method, method_param));
        cvReleaseMat( &new_responses );
        printf("\n");
    }

    mlp_response = cvCreateMat( 1, class_count, CV_32F );

    // compute prediction error on train and test data
    for( i = 0; i < nsamples_all; i++ )
    {
        int best_class;
        CvMat sample;
        cvGetRow( data, &sample, i );
        CvPoint max_loc = {0,0};
        mlp.predict( &sample, mlp_response );
        cvMinMaxLoc( mlp_response, 0, 0, 0, &max_loc, 0 );
        best_class = max_loc.x + 'A';

        int r = fabs((double)best_class - responses->data.fl[i]) < FLT_EPSILON ? 1 : 0;

        if( i < ntrain_samples )
            train_hr += r;
        else
            test_hr += r;
    }

    test_hr /= (double)(nsamples_all-ntrain_samples);
    train_hr /= (double)ntrain_samples;
    printf( "Recognition rate: train = %.1f%%, test = %.1f%%\n",
            train_hr*100., test_hr*100. );

    // Save classifier to file if needed
    if( filename_to_save )
        mlp.save( filename_to_save );

    cvReleaseMat( &mlp_response );
    cvReleaseMat( &data );
    cvReleaseMat( &responses );

    return 0;
}

static
int build_knearest_classifier( char* data_filename, int K )
{
    const int var_count = 16;
    CvMat* data = 0;
    CvMat train_data;
    CvMat* responses;

    int ok = read_num_class_data( data_filename, 16, &data, &responses );
    int nsamples_all = 0, ntrain_samples = 0;
    //int i, j;
    //double /*train_hr = 0,*/ test_hr = 0;
    CvANN_MLP mlp;

    if( !ok )
    {
        printf( "Could not read the database %s\n", data_filename );
        return -1;
    }

    printf( "The database %s is loaded.\n", data_filename );
    nsamples_all = data->rows;
    ntrain_samples = (int)(nsamples_all*0.8);

    // 1. unroll the responses
    printf( "Unrolling the responses...\n");
    cvGetRows( data, &train_data, 0, ntrain_samples );

    // 2. train classifier
    CvMat* train_resp = cvCreateMat( ntrain_samples, 1, CV_32FC1);
    for (int i = 0; i < ntrain_samples; i++)
        train_resp->data.fl[i] = responses->data.fl[i];
    CvKNearest knearest(&train_data, train_resp);

    CvMat* nearests = cvCreateMat( (nsamples_all - ntrain_samples), K, CV_32FC1);
    float* _sample = new float[var_count * (nsamples_all - ntrain_samples)];
    CvMat sample = cvMat( nsamples_all - ntrain_samples, 16, CV_32FC1, _sample );
    float* true_results = new float[nsamples_all - ntrain_samples];
    for (int j = ntrain_samples; j < nsamples_all; j++)
    {
        float *s = data->data.fl + j * var_count;

        for (int i = 0; i < var_count; i++)
        {
            sample.data.fl[(j - ntrain_samples) * var_count + i] = s[i];
        }
        true_results[j - ntrain_samples] = responses->data.fl[j];
    }
    CvMat *result = cvCreateMat(1, nsamples_all - ntrain_samples, CV_32FC1);
    knearest.find_nearest(&sample, K, result, 0, nearests, 0);
    int true_resp = 0;
    int accuracy = 0;
    for (int i = 0; i < nsamples_all - ntrain_samples; i++)
    {
        if (result->data.fl[i] == true_results[i])
            true_resp++;
        for(int k = 0; k < K; k++ )
        {
            if( nearests->data.fl[i * K + k] == true_results[i])
            accuracy++;
        }
    }

    printf("true_resp = %f%%\tavg accuracy = %f%%\n", (float)true_resp / (nsamples_all - ntrain_samples) * 100,
                                                      (float)accuracy / (nsamples_all - ntrain_samples) / K * 100);

    delete[] true_results;
    delete[] _sample;
    cvReleaseMat( &train_resp );
    cvReleaseMat( &nearests );
    cvReleaseMat( &result );
    cvReleaseMat( &data );
    cvReleaseMat( &responses );

    return 0;
}

static
int build_nbayes_classifier( char* data_filename )
{
    const int var_count = 16;
    CvMat* data = 0;
    CvMat train_data;
    CvMat* responses;

    int ok = read_num_class_data( data_filename, 16, &data, &responses );
    int nsamples_all = 0, ntrain_samples = 0;
    //int i, j;
    //double /*train_hr = 0, */test_hr = 0;
    CvANN_MLP mlp;

    if( !ok )
    {
        printf( "Could not read the database %s\n", data_filename );
        return -1;
    }

    printf( "The database %s is loaded.\n", data_filename );
    nsamples_all = data->rows;
    ntrain_samples = (int)(nsamples_all*0.5);

    // 1. unroll the responses
    printf( "Unrolling the responses...\n");
    cvGetRows( data, &train_data, 0, ntrain_samples );

    // 2. train classifier
    CvMat* train_resp = cvCreateMat( ntrain_samples, 1, CV_32FC1);
    for (int i = 0; i < ntrain_samples; i++)
        train_resp->data.fl[i] = responses->data.fl[i];
    CvNormalBayesClassifier nbayes(&train_data, train_resp);

    float* _sample = new float[var_count * (nsamples_all - ntrain_samples)];
    CvMat sample = cvMat( nsamples_all - ntrain_samples, 16, CV_32FC1, _sample );
    float* true_results = new float[nsamples_all - ntrain_samples];
    for (int j = ntrain_samples; j < nsamples_all; j++)
    {
        float *s = data->data.fl + j * var_count;

        for (int i = 0; i < var_count; i++)
        {
            sample.data.fl[(j - ntrain_samples) * var_count + i] = s[i];
        }
        true_results[j - ntrain_samples] = responses->data.fl[j];
    }
    CvMat *result = cvCreateMat(1, nsamples_all - ntrain_samples, CV_32FC1);
    nbayes.predict(&sample, result);
    int true_resp = 0;
    //int accuracy = 0;
    for (int i = 0; i < nsamples_all - ntrain_samples; i++)
    {
        if (result->data.fl[i] == true_results[i])
            true_resp++;
    }

    printf("true_resp = %f%%\n", (float)true_resp / (nsamples_all - ntrain_samples) * 100);

    delete[] true_results;
    delete[] _sample;
    cvReleaseMat( &train_resp );
    cvReleaseMat( &result );
    cvReleaseMat( &data );
    cvReleaseMat( &responses );

    return 0;
}

static
int build_svm_classifier( char* data_filename, const char* filename_to_save, const char* filename_to_load )
{
    CvMat* data = 0;
    CvMat* responses = 0;
    CvMat* train_resp = 0;
    CvMat train_data;
    int nsamples_all = 0, ntrain_samples = 0;
    int var_count;
    CvSVM svm;

    int ok = read_num_class_data( data_filename, 16, &data, &responses );
    if( !ok )
    {
        printf( "Could not read the database %s\n", data_filename );
        return -1;
    }
    ////////// SVM parameters ///////////////////////////////
    CvSVMParams param;
    param.kernel_type=CvSVM::LINEAR;
    param.svm_type=CvSVM::C_SVC;
    param.C=1;
    ///////////////////////////////////////////////////////////

    printf( "The database %s is loaded.\n", data_filename );
    nsamples_all = data->rows;
    ntrain_samples = (int)(nsamples_all*0.1);
    var_count = data->cols;

    // Create or load Random Trees classifier
    if( filename_to_load )
    {
        // load classifier from the specified file
        svm.load( filename_to_load );
        ntrain_samples = 0;
        if( svm.get_var_count() == 0 )
        {
            printf( "Could not read the classifier %s\n", filename_to_load );
            return -1;
        }
        printf( "The classifier %s is loaded.\n", filename_to_load );
    }
    else
    {
        // train classifier
        printf( "Training the classifier (may take a few minutes)...\n");
        cvGetRows( data, &train_data, 0, ntrain_samples );
        train_resp = cvCreateMat( ntrain_samples, 1, CV_32FC1);
        for (int i = 0; i < ntrain_samples; i++)
            train_resp->data.fl[i] = responses->data.fl[i];
        svm.train(&train_data, train_resp, 0, 0, param);
    }

    // classification
    std::vector<float> _sample(var_count * (nsamples_all - ntrain_samples));
    CvMat sample = cvMat( nsamples_all - ntrain_samples, 16, CV_32FC1, &_sample[0] );
    std::vector<float> true_results(nsamples_all - ntrain_samples);
    for (int j = ntrain_samples; j < nsamples_all; j++)
    {
        float *s = data->data.fl + j * var_count;

        for (int i = 0; i < var_count; i++)
        {
            sample.data.fl[(j - ntrain_samples) * var_count + i] = s[i];
        }
        true_results[j - ntrain_samples] = responses->data.fl[j];
    }
    CvMat *result = cvCreateMat(1, nsamples_all - ntrain_samples, CV_32FC1);

    printf("Classification (may take a few minutes)...\n");
    double t = (double)cvGetTickCount();
    svm.predict(&sample, result);
    t = (double)cvGetTickCount() - t;
    printf("Prediction type: %gms\n", t/(cvGetTickFrequency()*1000.));

    int true_resp = 0;
    for (int i = 0; i < nsamples_all - ntrain_samples; i++)
    {
        if (result->data.fl[i] == true_results[i])
            true_resp++;
    }

    printf("true_resp = %f%%\n", (float)true_resp / (nsamples_all - ntrain_samples) * 100);

    if( filename_to_save )
        svm.save( filename_to_save );

    cvReleaseMat( &train_resp );
    cvReleaseMat( &result );
    cvReleaseMat( &data );
    cvReleaseMat( &responses );

    return 0;
}

int main( int argc, char *argv[] )
{
    char* filename_to_save = 0;
    char* filename_to_load = 0;
    char default_data_filename[] = "./letter-recognition.data";
    char* data_filename = default_data_filename;
    int method = 0;

    int i;
    for( i = 1; i < argc; i++ )
    {
        if( strcmp(argv[i],"-data") == 0 ) // flag "-data letter_recognition.xml"
        {
            i++;
            data_filename = argv[i];
        }
        else if( strcmp(argv[i],"-save") == 0 ) // flag "-save filename.xml"
        {
            i++;
            filename_to_save = argv[i];
        }
        else if( strcmp(argv[i],"-load") == 0) // flag "-load filename.xml"
        {
            i++;
            filename_to_load = argv[i];
        }
        else if( strcmp(argv[i],"-boost") == 0)
        {
            method = 1;
        }
        else if( strcmp(argv[i],"-mlp") == 0 )
        {
            method = 2;
        }
        else if ( strcmp(argv[i], "-knearest") == 0)
    {
        method = 3;
    }
    else if ( strcmp(argv[i], "-nbayes") == 0)
    {
        method = 4;
    }
    else if ( strcmp(argv[i], "-svm") == 0)
    {
        method = 5;
    }
        else
            break;
    }

    if( i < argc ||
        (method == 0 ?
        build_rtrees_classifier( data_filename, filename_to_save, filename_to_load ) :
        method == 1 ?
        build_boost_classifier( data_filename, filename_to_save, filename_to_load ) :
        method == 2 ?
        build_mlp_classifier( data_filename, filename_to_save, filename_to_load ) :
        method == 3 ?
        build_knearest_classifier( data_filename, 10 ) :
        method == 4 ?
        build_nbayes_classifier( data_filename) :
        method == 5 ?
        build_svm_classifier( data_filename, filename_to_save, filename_to_load ):
        -1) < 0)
    {
        help();
    }
    return 0;
}
