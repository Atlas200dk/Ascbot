# coding: utf-8
import sys
sys.path.append('/home/jlm/caffe/python')
import numpy as np
import caffe
import os

deploy = '/home/jlm/jianglm_workspaces/Resnet18_class/Resnet18_class_deploy.prototxt'
caffe_model = '/home/jlm/jianglm_workspaces/Resnet18_class/Resnet18_table_weights/2+2_table_nj_iter_50000.caffemodel'

labels_filename = '/home/jlm/jianglm_workspaces/Resnet18_class/label_class.txt' 
test_image_list = "/home/jlm/jianglm_workspaces/Resnet18_class/Table_data/Table_anti-collision_data_83080_test_list.txt"
test_image_dir = '/home/jlm/jianglm_workspaces/Resnet18_class/Table_data/Table_anti-collision_data_83080_test'

result_path = '/home/jlm/jianglm_workspaces/Resnet18_class/test_result/83080_test.txt'

mean_value = [108,105,121]
image_size = (224, 224)

crash_sum = 0.0
falling_sum = 0.0
correct_danger0 = 0
correct_danger1 = 0
correct_danger2 = 0
correct_safe0 = 0
correct_safe1 = 0
car_correct = 0

danger0_danger1 = 0
danger0_danger2 = 0
danger1_danger2 = 0
safe0_safe1 = 0


caffe.set_mode_gpu()
net = caffe.Net(deploy,caffe_model,caffe.TEST)
net.blobs['data'].reshape(1, 3, *image_size)

transformer = caffe.io.Transformer({'data': net.blobs['data'].data.shape})  
transformer.set_transpose('data', (2,0,1))   
transformer.set_mean('data', np.array(mean_value))
transformer.set_raw_scale('data', 255)   
transformer.set_channel_swap('data', (2,1,0))

with open(test_image_list, 'r') as f_test, open(result_path, "a+") as f_out:
   for line in f_test.readlines():
      filename = line[:-1]
      filename_save = filename
      filename = os.path.join(test_image_dir, filename)
      # print (filename)
      image = caffe.io.load_image(filename, True)
      transformed_image = transformer.preprocess('data', image)
      # print(transformed_image.shape)
      # print(image.shape)
      net.blobs['data'].data[...] = transformed_image

      output = net.forward()

      labels = np.loadtxt(labels_filename, str, delimiter='\t') 
      prob= net.blobs['prob'].data[0].flatten()   #取出最后一层（prob）属于某个类别的概率值  data[0].flatten()
      order=prob.argsort()[3]  #从小到大排列，2分类网络:[0][1]

      # print (filename_save + ':' + labels[order])
      print (('{:<55}'.format(filename_save)) + ' is ' + labels[order])
      print ('------------------------------------------------')
      f_out.writelines(filename_save + ' ' + labels[order] + '\n')



with open(result_path,'r') as f:
   for fn in f.readlines():
      file = fn[:-1]
      file_correct = file.split('_')[1]
      file_result = file.split(' ')[1]
      if file_correct == 'danger0':
         falling_sum += 1
         if file_result == 'danger0':
            correct_danger0 += 1
            car_correct += 1
         elif file_result == 'danger1':
            car_correct += 1
            danger0_danger1 += 1
         elif file_result == 'danger2' :
            car_correct += 1
            danger0_danger2 += 1
         
      if file_correct == 'danger1':
         crash_sum += 1
         if file_result == 'danger1':
            correct_danger1 += 1
            car_correct += 1
         elif file_result == 'danger2' :
            car_correct += 1
            danger1_danger2 += 1
         elif file_result == 'danger0' :
            car_correct += 1
            danger0_danger1 += 1
         
      elif file_correct == 'danger2':
         crash_sum += 1
         if file_result == 'danger2':
            correct_danger2 += 1
            car_correct += 1
         elif file_result == 'danger1' :
            car_correct += 1
            danger1_danger2 += 1
         elif file_result == 'danger0' :
            car_correct += 1
            danger0_danger2 += 1

      elif file_correct == 'safe0':
         falling_sum += 1
         if file_result == 'safe0':
            correct_safe0 += 1
            car_correct += 1
         elif file_result == 'safe1':
            car_correct += 1
            safe0_safe1 += 1
         
      elif file_correct == 'safe1':
         crash_sum += 1
         if file_result == 'safe1':
            correct_safe1 += 1
            car_correct += 1
         elif file_result == 'safe0':
            car_correct += 1  
            safe0_safe1 += 1    


print('test sum :', crash_sum + falling_sum, '\n')
print ('accuracy for algorithm:', (correct_danger0 + correct_danger1 + correct_danger2 + correct_safe0 + correct_safe1 )/(crash_sum + falling_sum), '\n')
# print ('accuracy for algorithm:', (correct_danger0 + correct_danger1 + correct_danger2 + correct_safe0 + correct_safe1 ), '\n')
print('N_falling accuracy :',(correct_danger0 + correct_safe0) / falling_sum ,'\n')
print('N_crash accuracy :', (correct_danger1 + correct_danger2 + correct_safe1) / crash_sum,'\n')
print('accuracy for car :', car_correct / (crash_sum + falling_sum),'\n')

print('danger0 - danger1 : ', danger0_danger1, '\n')
print('danger0 - danger2 : ', danger0_danger2, '\n')
print('danger1 - danger2 : ', danger1_danger2, '\n')
print('safe0 - safe1 : ', safe0_safe1 , '\n')

with open(result_path, 'a+') as f:
   f.write('test sum :' + str(crash_sum + falling_sum) + '\n')
   f.write('accuracy for algorithm:' + str((correct_danger0 + correct_danger1 + correct_danger2 + correct_safe0 + correct_safe1 )/(crash_sum + falling_sum)) +  '\n')
   f.write('N_crash accuracy :' + str((correct_danger1 + correct_danger2 + correct_safe1) / crash_sum) + '\n')
   f.write('N_falling accuracy :' + str((correct_danger0 + correct_safe0) / falling_sum) + '\n')
   f.write('accuracy for car :' + str(car_correct / (crash_sum + falling_sum)) + '\n')
   f.write('danger1 - danger2 : ' +  str(danger1_danger2) + '\n')
   f.write('danger0 - danger1 : ' +  str(danger0_danger1) + '\n')
   f.write('danger0 - danger2 : ' +  str(danger0_danger2) + '\n')
   f.write('safe0 - safe1 : ' + str(safe0_safe1) , '\n')
