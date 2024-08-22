import bcrypt from 'bcryptjs';
import { LowSync } from 'lowdb';
import { JSONFileSync } from 'lowdb/node';

const defaultData = { users: [] };
const db = new LowSync(new JSONFileSync('database.json'), defaultData);

db.read();
const saltRounds = 10;

const users = [
  {  pin: bcrypt.hashSync('1111', saltRounds), id: 12032, name: 'Llama User', role: "Administrator" },
  {  pin: bcrypt.hashSync('7500', saltRounds), id: 11313, name: 'Jonathan Turnage', role: "Operator"  }
];

db.data.users.push(...users);
db.write();
