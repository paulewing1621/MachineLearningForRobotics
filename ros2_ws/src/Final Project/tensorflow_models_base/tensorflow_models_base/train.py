
import tensorflow_models_base.venv_hack

import tensorflow as tf
import numpy as np
import os

from tensorflow_models_base.parser import ParseArgs
from tensorflow_models_base.sampler import UniformSampler
from tensorflow_models_base.reader import PNGReader
from tensorflow_models_base.model import create_cnn_model

class Train:
    def __init__(self):
        self.settings = ParseArgs()
        self.dataset = PNGReader(self.settings.train_file, self.settings.train_root, self.settings.val_file, self.settings.val_root)
        self.sampler = UniformSampler(self.dataset)
        self.model = create_cnn_model(32, 32, 3, self.settings.num_class, self.dataset.mean, self.dataset.std, self.settings.dropout)
        self.optimizer = tf.keras.optimizers.Adam(learning_rate=self.settings.lr)
        self.train_loss = tf.keras.losses.SparseCategoricalCrossentropy()
        self.train_loss_metric = tf.keras.metrics.Mean('train_loss',dtype=tf.float32)
        self.train_accuracy_metric = tf.keras.metrics.SparseCategoricalAccuracy('train_accuracy',dtype=tf.float32)
        self.val_loss = tf.keras.losses.SparseCategoricalCrossentropy()
        self.val_loss_metric = tf.keras.metrics.Mean('val_loss',dtype=tf.float32)
        self.val_accuracy_metric = tf.keras.metrics.SparseCategoricalAccuracy('val_accuracy',dtype=tf.float32)

        self.model(np.zeros([1,32,32,3]))
        self.model.summary()
    
    @tf.function
    def train_step(self, x, y):
        with tf.GradientTape() as tape:
            pred = self.model(x)
            loss = self.train_loss(y, pred)
        grads = tape.gradient(loss, self.model.trainable_variables)
        self.optimizer.apply_gradients(zip(grads, self.model.trainable_variables))
        self.train_loss_metric.update_state(loss)
        self.train_accuracy_metric.update_state(y, pred)
        
    @tf.function
    def val_step(self, x, y):
        pred = self.model(x, training=False)
        loss = self.val_loss(y, pred)
        self.val_loss_metric.update_state(loss)
        self.val_accuracy_metric.update_state(y, pred)

    def train(self):
        best_acc = 10000
        self.train_writer = tf.summary.create_file_writer(self.settings.tensorboard_dir+'/train')
        self.val_writer = tf.summary.create_file_writer(self.settings.tensorboard_dir+'/val')
        with self.train_writer.as_default():
            tf.summary.graph(self.train_step.get_concrete_function(np.ones([1,32,32,3]),np.ones(1)).graph)

        step = 0
        while (step < self.settings.max_iter):
        #for step in range(self.settings.max_iter):
            x,y = self.sampler.sample_train_batch(self.settings.bs)
            #print(y.shape, x.shape)
            self.train_step(x,y)
            step += 1
            if step%25==0:
                x,y = self.sampler.sample_val_batch(self.settings.bs)
                self.val_step(x,y) 
            if step%100==0:
                print('Train loss : ', float(self.train_loss_metric.result()), 'train accuracy : ', float(self.train_accuracy_metric.result())*100,'%')
                print('Val loss   : ', float(self.val_loss_metric.result()),  'val accuracy   : ', float(self.val_accuracy_metric.result())*100,'%')
                with self.train_writer.as_default():
                    tf.summary.scalar('loss', self.train_loss_metric.result(), step=step)
                    tf.summary.scalar('accuracy', self.train_accuracy_metric.result(), step=step)
                with self.val_writer.as_default():
                    tf.summary.scalar('loss', self.val_loss_metric.result(), step=step)
                    tf.summary.scalar('accuracy', self.val_accuracy_metric.result(), step=step)
                self.train_loss_metric.reset_state()
                self.train_accuracy_metric.reset_state()
                self.val_loss_metric.reset_state()
                self.val_accuracy_metric.reset_state()
            if step%1000==0:
                self.model.save(os.path.join(self.settings.model_dir,'step_'+str(step)+'.keras'))
        print('Training done ! ^_^')

def main(args=None):
    T = Train()
    T.train()

if __name__ == '__main__':
    main()
