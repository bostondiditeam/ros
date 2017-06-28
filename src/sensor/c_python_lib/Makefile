#before . /main you must run the command below
#export LD_LIBRARY_PATH=/home/jidou/anaconda3/envs/py3.5/lib:$LD_LIBRARY_PATH

INC=/home/jidou/anaconda3/envs/py3.5/include/python3.5m/
LIB=/home/jidou/anaconda3/envs/py3.5/lib/

predict_test: predict_test.cpp
	g++ -o predict_test  predict_test.cpp -I$(INC)/ -L$(LIB)  -lpython3.5m

clean:
	rm -rf predict_test.o predict_test
